//Monitor the tracker ASIC front-end buffers, and generate event triggers and read
//commands during a data acquisition run.
//R. Johnson
//Modifed 6/10/2013 not to use buffer clear signals from the front-end boards.
//Modified 8/8/2013 to make the HoldError one clock cycle in length
//Modified 8/12/2013 to improve monitoring of front-end buffers
//Modified 8/17/2013 to add parity bit to outgoing trigger signal
//Modified 8/19/2013 to add trigger counter
//Modified 8/27/2013 to add ResetBuf argument
//Modified 9/27/2013 to rewrite the trigger generation part
//Modified 10/25/2013 to send energy detector read commands at same time as TKR send commands
//Modified 3/21/2014 to simplify new buffer management scheme
//
//When a TrigIn pulse comes in a trigger is issued if not all buffers are busy,
//otherwise it is ignored.  This version assumes that all buffers are busy if
//the number of triggers sent is 3 greater than the number of events received.
//An ASIC read command can be sent immediately after the trigger, as the ASICs
//will buffer it.  The command for the front-end tracker boards to send the event
//to the event builder, however, may have to be held if the event builder buffers
//are backed up.

module pCT_BufMngr (HoldError,HoldExt,ReadTrig, Reset, Clock, TrigIn, TrigOut, Cmd, CmdE, BusyHold, TrigSend, FEBtrgHld, NoSend, BufClr, Trig, BufMngrType, NRead, Nmissed);
output HoldError;		//Signal that the trigger and buffering have been stuck for more than 1 millisecond
input HoldExt;			//External signal to hold the trigger
input Reset;        	//Full reset
input Clock;        	//System clock 
input TrigIn;       	//Trigger pulse
output BusyHold;        //For monitoring the length of time that the trigger is on hold
output TrigOut;     	//Output trigger, 4-bit serial stream
output Cmd;         	//Outgoing read commands for the tracker
output CmdE;			//Outgoing read commands for the energy digitizers
output TrigSend;        //1-clock signal that a send command is going out
input FEBtrgHld;        //Hold trigger to prevent overflow of front-end board buffers
input NoSend;           //Stop pulling events from the tracker board to avoid overflow of local buffers
input [11:0] BufClr;	//Buffer free status for each Tracker FPGA.  Accept new trigger only if all 1's
output Trig;			//One clock pulse for each trigger issued
input BufMngrType;		//Set to 1 to use the BufClr signals
output [31:0] NRead, Nmissed;
output ReadTrig;		//For monitoring

reg Cmd, BusyPl;
wire BusyHold;
reg TrigOut, TrigSend;
reg HoldError;
reg [31:0] NRead;

//Enumeration of states for queuing the read commands
parameter [2:0] Look = 3'b001;  //Wait for trigger
parameter [2:0] Queu = 3'b010;  //Queue a read command
parameter [2:0] Isue = 3'b100;  //Issue a read command
reg [2:0] StateRead, NextStateRead;

reg [1:0] NextBuffer;             //To keep track of buffers to which to send triggers
reg ReadTrig;                     //Flag to keep BusyHold true while trigger bits are going out
reg OnHold;						  //Set true for monitoring if triggers are being held due to full front-end buffers
reg [15:0] HoldTime;			  //Count time for which the buffers are full and the trigger is on hold

reg Trig;             //1-clock long trigger pulse
reg [1:0] TrigTag;    //Trigger tag

wire Busy;             //Flag to indicate when a new read command can be sent
reg BusyOld;           //Flag to catch the falling edge of Busy
reg Inhibit;           //Flag to hold off issuing the next read command
reg [2:0] NBufRdFull;  //Buffer occupancy for the read command state machine

reg [1:0] Rptr, Wptr;                      //Read and write pointers for the address
reg [1:0] AddrQ0, AddrQ1, AddrQ2, AddrQ3;  //Queue up to 4 read addresses. 
reg [1:0] TagRead;                         //Tag to send out with the read command
reg TrigRead;                              //Signal to start a new read command flowing
wire FEBtrgHld;                            //Signal to hold off on sending read commands due to DAQ buffer occupancy

//Debug output 
//initial begin
//  $display("Time     StateTrg  TrigIn TrigOut BsyHld  BuffBsy UnDoBf Busy BusyOld     Inhibit Rptr Wptr TrgRd TgRd StRd Cmd Trig TrigTag FEBtrgHld NBfFl TrgRd nPend BsyPl StatePl CntPl");
//end

//always @ (posedge Clock) begin
//	$display("%g\t     %b     %b      %b     %b        %b     %b      %b       %b        %b      %h    %h     %b    %b   %b  %b   %b     %b     %b    %h    %b   %b   %b   %b   %b",
//         $time,StateTrig,TrigIn,TrigOut,Busy,BufferBusy,
//          UnDoBuf,Busy,BusyOld,Inhibit,Rptr,Wptr,TrigRead,TagRead,StateRead,Cmd,Trig,TrigTag,FEBtrgHld,NBufRdFull,TrigRead,nPending,BusyPl,StatePl,CntPl);
//end

always @ (posedge Trig)
begin
	$display("%g\t pCT_BufMngr: Trigger issued with Tag=%b, SendHold=%b, nPending=%d, Busy=%b",$time,TrigTag,NoSend,nPending,Busy);
end

//Instantiate the module used to generate the read command strings
GenRead3 GenRead3_U(Reset, Clock, BusyPl, TrigRead, TagRead, Busy, CmdRd3);

//Keep count of how many events need to be pulled from the front-end boards
reg [3:0] nPending;
reg TrigRead2;
always @ (posedge Clock) begin
	if (Reset) begin
		nPending <= 0;
		TrigRead2 <= 1'b0;
	end else begin
		TrigRead2 <= TrigRead;
		if (TrigRead && !(StatePl==DonePl)) nPending <= nPending + 1;
		if ((StatePl == DonePl) && !TrigRead) nPending <= nPending - 1;
	end
end

//State machine to issue commands to pull events from the tracker boards
//and energy detector boards.
//We can issue the send command immediately after the read command goes to
//the ASICs, because the front-end FPGA will buffer the send commands.  However,
//we have to wait and grab the command output line when it is not being used
//for ASIC read commands.
parameter [4:0] WaitPl = 5'b00001;
parameter [4:0] SendPl = 5'b00010;
parameter [4:0] Snd2Pl = 5'b00100;
parameter [4:0] DonePl = 5'b01000;
parameter [4:0] PausPl = 5'b10000;

reg [9:0] RdCmd;
reg [4:0] StatePl, NextStatePl;
reg [3:0] CntPl;
reg [16:0] RdCmdE;
reg [2:0] TagE;
reg CmdE;
always @ (StatePl or nPending or NoSend or Busy or TrigRead or TrigRead2 or CmdRd3 or RdCmd or CntPl or RdCmdE or StateRead or NextStateRead) begin
	case (StatePl)
		WaitPl:	begin
					Cmd = CmdRd3;
					if (nPending >0 && StateRead != Isue && NextStateRead != Isue && !Busy && !TrigRead && !TrigRead2 && !NoSend) begin
						NextStatePl = SendPl;
						BusyPl = 1'b1;
						CmdE = 1'b1;  //Send the energy detector start bit 1 clock cycle early, to save time
					end else begin
						NextStatePl = WaitPl;
						BusyPl = 1'b0;
						CmdE = 1'b0;
					end
				end
		SendPl: begin
					CmdE = RdCmdE[16];
					Cmd = RdCmd[9];
					if (CntPl == 9) NextStatePl = Snd2Pl;
					else NextStatePl = SendPl;
					BusyPl = 1'b1;
				end
		Snd2Pl:	begin
					Cmd = CmdRd3;
					CmdE = RdCmdE[16];
					if (CntPl == 15) NextStatePl = DonePl;
					else NextStatePl = Snd2Pl;
					BusyPl = 1'b0;
				end
		DonePl: begin
					CmdE = RdCmdE[16];
					Cmd = CmdRd3; 
					NextStatePl = PausPl;
					BusyPl = 1'b0;
				end
		PausPl:	begin
					CmdE = 1'b0;
					Cmd = CmdRd3;
					NextStatePl = WaitPl;
					BusyPl = 1'b0;
				end
		default: begin
					Cmd = CmdRd3;
					CmdE = 1'b0;
					BusyPl = 1'b0;
					NextStatePl = WaitPl;
				 end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		StatePl <= WaitPl;
		TagE <= 0;
		TrigSend <= 1'b0;
	end else begin
		StatePl <= NextStatePl;
		case (StatePl)
			WaitPl: begin
						RdCmd <= 10'b1111101111;    //Tracker send command
						RdCmdE <= {13'b1111010100000,TagE,TagE[0]^TagE[1]^TagE[2]};  //Energy detector read command, not including start bit
						CntPl <= 0;
						//if (nPending > 0) $display("%g\t pCT_BufMngr:  nPending=%d, NoSend=%b, Busy=%b, TrigRead=%b, TrigRead2=%b",$time,nPending,NoSend,Busy,TrigRead,TrigRead2);
					end
			SendPl: begin
						if (CntPl==0) $display("%g\t pCT_BufMngr: Issue send command.  nPending=%d",$time,nPending);
						CntPl <= CntPl + 1;
						RdCmd <= {RdCmd[8:0],1'b0};
						RdCmdE <= {RdCmdE[15:0],1'b0};
					end
			Snd2Pl:	begin
						RdCmdE <= {RdCmdE[15:0],1'b0};
						CntPl <= CntPl + 1;
					end
			DonePl: begin
						RdCmdE <= {RdCmdE[15:0],1'b0};
						$display("%g\t pCT_BufMngr: energy detector read command issued for tag %d.",$time,TagE);
						TagE <= TagE + 1;
						TrigSend <= 1'b1;
					end
			PausPl:	begin
						TrigSend <= 1'b0;
					end
		endcase
	end
end

//State machine for issuing the read commands for each trigger.
//Note that successive triggers cannot be less than 3 clock cycles in spacing.
//There has to be a queue for the read commands because more triggers may come
//in while the serial command stream is going out.
always @ (StateRead or Trig or Inhibit or NBufRdFull)
begin
  case (StateRead)
    Look: if (Trig)
            NextStateRead= Queu;
          else if (!Inhibit && NBufRdFull != 0) 
            NextStateRead= Isue;
          else
            NextStateRead= Look;
    Queu: NextStateRead=Look;
    Isue: if (Trig)
            NextStateRead=Queu;
          else
            NextStateRead=Look;
    default: NextStateRead=Look;
  endcase
end

always @ (posedge Clock)
begin
  if (Reset) begin
    StateRead <= Look;
    Rptr <= 0;
    Wptr <= 0;
    Inhibit <= 1'b0;
	NRead <= 0;
	TrigRead <= 1'b0;
	NBufRdFull <= 0;
  end else begin
    StateRead <= NextStateRead;
    case (StateRead)
      Look: begin
              TrigRead<=1'b0;
              BusyOld <= Busy;
              if (BusyOld && !Busy) Inhibit <= 0;
            end
      Queu: begin
			  TrigRead<=1'b0;
	          if (NBufRdFull==4) $display("%g\t pCT_BufMngr: read-command buffer overflow!",$time);
              Wptr <= Wptr + 1;
			  NBufRdFull <= NBufRdFull + 1;
              case (Wptr)
                2'b00: AddrQ0<= TrigTag;
                2'b01: AddrQ1<= TrigTag;
                2'b10: AddrQ2<= TrigTag;
                2'b11: AddrQ3<= TrigTag;
              endcase
            end
      Isue: begin      
              TrigRead <= 1'b1;            //Issue a read command here
			  NRead <= NRead + 1;
			  NBufRdFull <= NBufRdFull - 1;
              case (Rptr)
                2'b00: TagRead <= AddrQ0;
                2'b01: TagRead <= AddrQ1;
                2'b10: TagRead <= AddrQ2;
                2'b11: TagRead <= AddrQ3;
              endcase
			  $display("%g\t pCT_BufMngr: issue read.  Rptr=%h, Wptr=%h.  Addresses=%h %h %h %h. SendHold=%b",$time,Rptr,Wptr,AddrQ3,AddrQ2,AddrQ1,AddrQ0,NoSend);
              Rptr <= Rptr + 1;
              Inhibit <= 1;
            end
    endcase
  end
end

//Enumeration of states for the trigger monitor machine
parameter [4:0] Wait = 5'b00001;  //Wait for a trigger and send the first bit
parameter [4:0] Bit1 = 5'b00010;  //Generate the first tag bit
parameter [4:0] Bit2 = 5'b00100;  //Generate the second tag bit
parameter [4:0] Prty = 5'b01000;  //Send parity bit
parameter [4:0] DoIt = 5'b10000;  //Wait one clock cycle
reg [4:0] StateTrig, NextStateTrig;

//State machine for issuing triggers.  TrigIn is ignored if there is an external hold
//or if there is any possibility of all 4 front-end buffers being full.
always @ (StateTrig or TrigIn or FEBtrgHld or HoldExt or BufMngrType or NextBuffer or BufClr)
begin
  case (StateTrig)
      Wait: begin
				if (TrigIn) begin
					if (BufMngrType) begin
						if ((BufClr == 12'b111111111111) && !(HoldExt | FEBtrgHld)) NextStateTrig = Bit1;
						else NextStateTrig = Wait;
					end else begin
						if (!(HoldExt | FEBtrgHld)) NextStateTrig = Bit1;
						else NextStateTrig = Wait;
					end
				end else begin
					NextStateTrig = Wait;
				end
				ReadTrig = 1'b0;
			end
      Bit1: begin
				NextStateTrig = Bit2;
				ReadTrig = 1'b1;
			end
	  Bit2: begin
				NextStateTrig = Prty;
				ReadTrig = 1'b1;
			end
	  Prty: begin
				NextStateTrig = DoIt;
				ReadTrig = 1'b1;
			end
      DoIt: begin
				ReadTrig = 1'b0;
				NextStateTrig = Wait;   
			end
      default: begin NextStateTrig = Wait; ReadTrig = 1'b1; end
  endcase
end

reg [31:0] Nmissed;
always @ (posedge Clock)
begin
  if (Reset) begin
    NextBuffer<=2'b00;
    TrigOut <= 0;
    StateTrig <= Wait;
	OnHold <= 1'b0;
	HoldTime <= 0;
	Trig <= 1'b0;
	Nmissed <= 0;
  end else begin
    StateTrig <= NextStateTrig;
	if (HoldTime == 16'b1111111111111111) begin
		HoldError <= 1'b1;
		HoldTime <= HoldTime + 1;
	end else begin 
		HoldError <= 1'b0;
		if (OnHold & !HoldExt) HoldTime <= HoldTime + 1;
		else HoldTime <= 0;
	end
    case (StateTrig)
      Wait: begin
			  if (TrigIn) begin
				if (BufClr != 12'b111111111111) Nmissed <= Nmissed + 1;
			  end
			  if (NextStateTrig == Bit1) begin
				TrigOut <= 1'b1;
			  end else begin
				TrigOut <= 1'b0;
			  end
			  if (BufMngrType) begin
				  if (BufClr == 12'b111111111111 && !FEBtrgHld) begin
					OnHold <= 1'b0;
					if (OnHold) $display("%g\t pCT_BufMngr mode 1: releasing hold on trigger.  BufClr=%b, FEBtrgHld=%b, HoldExt=%b",$time,BufClr,FEBtrgHld,HoldExt);
				  end else begin
					OnHold <= 1'b1;
					if (!OnHold) $display("%g\t pCT_BufMngr mode 1: putting trigger on hold.  BufClr=%b, FEBtrgHld=%b, HoldExt=%b",$time,BufClr,FEBtrgHld,HoldExt);
				  end		
			  end else begin
				  if (!FEBtrgHld) begin
					OnHold <= 1'b0;
					if (OnHold) $display("%g\t pCT_BufMngr mode 0: releasing hold on trigger.  BufClr=%b, FEBtrgHld=%b, HoldExt=%b",$time,BufClr,FEBtrgHld,HoldExt);
				  end else begin
					OnHold <= 1'b1;
					if (!OnHold) $display("%g\t pCT_BufMngr mode 0: putting trigger on hold.  BufClr=%b, FEBtrgHld=%b, HoldExt=%b",$time,BufClr,FEBtrgHld,HoldExt);
				  end
			  end
            end
      Bit1: begin
              TrigOut <= NextBuffer[1];
			  TrigTag[1] <= NextBuffer[1];
            end
      Bit2: begin
              TrigOut <= NextBuffer[0];
			  TrigTag[0] <= NextBuffer[0];
            end
	  Prty:	begin
			  Trig <= 1'b1;
              NextBuffer <= NextBuffer + 1;
			  TrigOut <= ~(TrigTag[0]^TrigTag[1]);     //Parity bit going out
			end
	  DoIt: begin
			  Trig <= 1'b0;
			  TrigOut <= 1'b0;
			end
    endcase
  end
end

assign BusyHold = OnHold | ReadTrig | HoldExt;

endmodule