//Module to buffer the serial data stream from a single pCTFE64 chip and then
//output it into the serial output stream when ready
//R. Johnson
//April 7, 2014: change state vectors to be 1-hot and removed negative clock edge process

module DataMon (Chip, Clock, Reset, DataIn, EndRun, Send, Done, Ready, FCL, TagOut, Dout, BufCnt, Ready2, ParityOut);
input [3:0] Chip;       //ID of the chip being monitored
input Clock;            //System clock
input Reset;			//Hard system reset
input DataIn;           //Serial data stream
input EndRun;           //Signal to end the run
input Send;             //Signal to start sending cluster data out from next buffer
output [3:0] Done;      //Signal that the end of event read was reached for the corresponding buffer
output Ready;           //Held true when the chip has data available to output into the merged stream
output FCL;             //A flag to indicate presence of clusters
output [1:0] TagOut;    //Trigger tag of data in next buffer
output Dout;            //Serial output stream of cluster data
output [2:0] BufCnt;    //Status of the 8 buffers in this instance of DataMon
output Ready2;          //Current output buffer is ready for reading
output ParityOut;       //Parity of the data in the current buffer

wire [3:0] Chip;                  
reg [3:0] Done;
wire FCL;
wire [1:0] TagOut;
reg [2:0] BufCnt;
reg Dout;
wire Send;
reg Ready, Ready2, ParityOut, ErrBit;

reg ReadyOn, ReadyOff, DParity, PrtyChk;
reg [4:0] NCL;
wire [1:0] Tag;                    //Trigger tag extracted from the event stream

//reg [31:0] NEvtOut;               //Count number of events read out

reg [2:0] WritePtr;               //Next Buffer to write to
reg [2:0] ReadPtr;                //Next Buffer to read from

reg [5:0] Header;
reg [5:0] NumClus;        //Information extracted from the incoming data stream
//reg [11:0] Output0 [0:10];        //Buffer of all the cluster info from an event; the 0th entry is for the header
//reg [11:0] Output1 [0:10];        //The pCTFE64 chip can never output more than 10 clusters
//reg [11:0] Output2 [0:10];
//reg [11:0] Output3 [0:10];
//reg [11:0] Output4 [0:10];
reg [7:0] BufReady;               //A flag for each buffer to indicate that it is ready to read
reg [7:0] BufParity;              //Parity of the data in each output buffer
reg [7:0] PrtyErr;				  //Parity of the data in each input event (should be zero)
reg [11:0] DataWord;              //Temporary storage for shifting data in
reg [2:0] CntH;                   //Count header bits
reg [3:0] CntC;                   //Count cluster bits
reg [6:0] CntN;                   //Count bits in the number of clusters
reg [3:0] Addr;                   //Count clusters
reg PacketType;                   //Distinguish events from register readback

wire [6:0] addra, addrb;    	//addresses for the dual-port memory
reg ena, enb, wea;   			//enables for the memory

//Enumeration of states for the data monitor machine
parameter [7:0] Idle = 8'b00000001;   //Wait for start bit
parameter [7:0] Hdr1 = 8'b00000010;   //Get first bit of header
parameter [7:0] Hdrr = 8'b00000100;   //Shift out header
parameter [7:0] Nclu = 8'b00001000;   //Shift out cluster number
parameter [7:0] Init = 8'b00010000;   //Initiate data output
parameter [7:0] Shft = 8'b00100000;   //Shift out data
parameter [7:0] EOEV = 8'b01000000;   //End of event
parameter [7:0] Regs = 8'b10000000;   //Shift out register data
reg [7:0] State, NextState;

//Enumeration of states for the data sending machine
parameter [3:0]  Wait = 4'b0001;    //Wait for a start signal
parameter [3:0]  Hdro = 4'b0010;    //Stream out the header info
parameter [3:0]  Strm = 4'b0100;    //Stream out the cluster info
parameter [3:0]  Incr = 4'b1000;    //Increment the read pointer
reg [3:0] StateSend, NextStateSend;
reg [3:0] Scntc;
reg [3:0] Scntb;
reg [3:0] Scnth;

reg [11:0] OutBuf,OutBuf2;
wire [11:0] Word0, doa;
reg IncrBufCnt, DecrBufCnt, Decr2BufCnt;
//reg [31:0] NBuf0,NBuf1,NBuf2,NBuf3,NBuf4,NBuf5;

//Debug printout.  
//parameter Debug = 1'b1;
//initial begin
//  if (Chip == 4'b0011 && Debug) begin
//    $display(" time    State StateSend DataIn   Dout Header NumClus     DataWord  NCL Addr CntC OutBuf      Word0      OutBuf2   Scntc Scntb Done Ready ReadPtr WritePtr Incr Decr BufCnt Snd BfRdy Rd2 FCL TgOut ena enb wea   addra   addrb     doa bfprty prtout");
//  end
//end

//always @ (posedge Clock) begin
//  if (Chip == 4'b0011) begin
//    $display("%g\t %b      %b      %b        %b   %b  %b  %b %2d %2d   %2d %b %b %b %2d  %2d    %b   %b    %2d       %2d     %b     %b    %h    %b   %b  %b     %b  %b %b %b %b %b %b %b %b   %b",       $time,State,StateSend,DataIn,Dout,Header,NumClus,DataWord,NCL,Addr,CntC,OutBuf,Word0,OutBuf2,Scntc,Scntb,Done,Ready,ReadPtr,WritePtr,IncrBufCnt,DecrBufCnt,BufCnt,Send,BufReady,Ready2,FCL,TagOut,ena,enb,wea,addra,addrb,doa,BufParity,ParityOut);
//  end
//end

assign FCL = Word0[4:0] != 0;
assign Tag = Header[3:2];
assign TagOut = Word0[9:8];

//Keep track of the buffer occupancy in this instance. 
always @ (posedge Clock) 
begin
  if (Reset) begin
    BufCnt <= 0;
//	NBuf0<= 0;
//	NBuf1<= 0;
//	NBuf2<= 0;
//	NBuf3<= 0;
//	NBuf4<= 0;
//	NBuf5<= 0;
  end else begin
//   case (BufCnt)
//	  3'b000: NBuf0 <= NBuf0 + 1;
//	  3'b001: NBuf1 <= NBuf1 + 1;
//	  3'b010: NBuf2 <= NBuf2 + 1;
//	  3'b011: NBuf3 <= NBuf3 + 1;
//	  3'b100: NBuf4 <= NBuf4 + 1;
//	  3'b101: NBuf5 <= NBuf5 + 1;   //Should only increment this in an error condition
//	  default: NBuf0 <= NBuf0 + 1;	  
//	endcase
    if (IncrBufCnt && !(DecrBufCnt || Decr2BufCnt)) begin
	  BufCnt <= BufCnt + 1;
//	  if (BufCnt >= 7) $display("%g\t DataMon: buffer overflow!  BufCnt=%h",$time,BufCnt);
	end
	if ((DecrBufCnt || Decr2BufCnt) && !IncrBufCnt) begin
	  if (BufCnt == 0) begin
//	    $display("%g\t DataMon: buffer underflow!",$time);
	  end else BufCnt <= BufCnt - 1;
	end
  end
end

always @ (BufParity or PrtyErr or ReadPtr) 
begin
	ParityOut = BufParity[ReadPtr]; 
	ErrBit = PrtyErr[ReadPtr];
end

//State machine to send the cluster information out
always @ (StateSend or Send or Scntc or Scntb or Scnth or NCL or ReadPtr or BufReady or Word0 or OutBuf or OutBuf2)
begin
  case (StateSend)
    Wait: begin
			enb = 1;                    //Read the header from memory
			Dout = 0;
			Ready2 = BufReady[ReadPtr];
	        if (Send == 1'b1) 
	          if (Word0[4:0] == 0)
			    NextStateSend = Incr;
			  else
	            NextStateSend = Hdro;
		    else
		      NextStateSend = Wait;
		  end
	Hdro: begin
	        Ready2 = 1'b0;
			Dout = OutBuf[11];
	        if (Scnth == 11) begin
			  enb = 1;                  //Read the first cluster from memory
	          NextStateSend = Strm;
		    end else begin
			  enb = 0;
              if (Word0[4:0] == 0)
                NextStateSend = Incr;
              else			  
		        NextStateSend = Hdro;
			end
		  end
	Strm: begin
			if (Scntb==0 && Scntc==1) Dout = OutBuf[11]; else Dout = OutBuf2[11];
			if (Scntb == 11) enb = 1; else enb = 0;  //Read the next cluster from memory
	        Ready2 = 1'b0;
	        if (Scntc-1 == NCL && Scntb == 11) 
	          NextStateSend = Incr;
		    else
		      NextStateSend = Strm;
		  end
	Incr: begin
			Dout = OutBuf2[11];
			enb = 0;
	        Ready2 = 1'b0;
	        NextStateSend = Wait;
		  end
	default:	begin
					NextStateSend = Wait;
					Dout = 1'b0;
					Ready2 = 1'b0;
					enb = 1'b0;
				end
  endcase
end

always @ (posedge Clock)
begin
  if (Reset) begin
    StateSend <= Wait;
	ReadPtr <= 0;
	Scntc <= 0;
  end else begin
    StateSend <= NextStateSend;
	case (StateSend)
	  Wait: begin
			  OutBuf <= {OutBuf[10:0], 1'b0};  //Just to flush out buffer
			  OutBuf2 <= {OutBuf2[10:0], 1'b0};
              Scntb <= 0;
			  Scnth <= 0;
			  ReadyOff <= 0;
			  DecrBufCnt <= 0;
	        end
	  Hdro: begin
              if (Scnth == 0) begin
				OutBuf <= {Word0[5:0],Word0[7],ErrBit,Chip};   //Put together the chip header
				NCL <= Word0[3:0];
				Scntc <= Scntc + 1;
		      end else begin
				OutBuf <= {OutBuf[10:0], 1'b0};
		      end
		      Scnth <= Scnth + 1;
	        end
	  Strm: begin
			  OutBuf <= {OutBuf[10:0], 1'b0};
	          if (Scntb == 0) begin
				OutBuf2 <= Word0;
				Scntc <= Scntc + 1;
				Scntb <= Scntb + 1;
			  end else begin
			    OutBuf2 <= {OutBuf2[10:0], 1'b0};
			    if (Scntb == 11)
				  Scntb <= 0;
				else
			      Scntb <= Scntb + 1;
			  end
	        end
	  Incr: begin
	          ReadPtr <= ReadPtr + 1;
			  Scntc <= 0;
			  DecrBufCnt <= 1;
              if (BufCnt == 1) ReadyOff <= 1;
			  OutBuf2 <= {OutBuf2[10:0], 1'b0};
			end
	endcase
  end
end

//Update the output status flag on every clock edge
always @ (posedge Clock)
begin
  if (Reset) begin
    Ready <= 0;
  end else begin
    if (ReadyOn) begin
	  Ready <= 1;
	end else if (ReadyOff) begin
	  Ready <= 0;
	end
  end
end

//State machine for monitoring the incoming data stream
always @ (State or DataIn or CntH or CntC or CntN or NumClus or PacketType or Addr)
begin
  case(State)
    Idle: begin
			ena = 0; wea = 0;
			if (DataIn) NextState = Hdr1;
			else NextState = Idle;
		  end
	Hdr1: begin NextState = Hdrr; ena = 0; wea = 0; end
    Hdrr: begin
			if (CntH==4) begin
				if (PacketType)
					NextState = Regs;
				else
					NextState = Nclu;
			end else NextState = Hdrr;
		    ena = 0; wea = 0;
		  end
    Nclu: begin
			if (CntN==5) NextState = Init;
			else NextState = Nclu;
			ena = 0; wea = 0;
		  end
    Regs: begin
			if (CntN==66) NextState = Idle;
			else NextState = Regs;
			ena = 0; wea = 0;
		  end
    Init: begin
			if (NumClus[4:0]==0) begin
				NextState = EOEV;
			end else begin
				NextState = Shft;
			end
			ena = 1; wea = 1;
		  end
    Shft: begin
			if (Addr-1 == NumClus[4:0]) NextState = EOEV;
			else NextState = Shft;
			if (CntC==12) wea = 1; else wea = 0;
			ena = 1;
		  end
    EOEV: begin
			if (DataIn) NextState= Hdr1;
			else NextState = Idle;
			ena = 0; wea = 0;
		  end
	default:	begin
					NextState = Idle;
					ena = 0; wea = 0;
				end
  endcase
end

always @ (posedge Clock)
begin
  if (Reset) begin
    State <= Idle;
	WritePtr <= 0;
	IncrBufCnt <= 0;
	Decr2BufCnt <= 0;
	BufReady <= 0;
  end else begin
    State <= NextState;
    case(State)
      Idle: begin
              CntN <= 0;
			  Header <= {Header[4:0], DataIn};
              DataWord <= {DataWord[10:0], DataIn};
              Done<= 4'b0000;
			  ReadyOn <= 0;
			  PrtyChk <= 1;
			  Addr <= 0;
            end
	  Hdr1: begin
			  PrtyChk <= PrtyChk^DataIn;
			  Header <= {Header[4:0], DataIn};
			  DataWord <= {DataWord[10:0], DataIn};
			  DParity <= Chip[0]^Chip[1]^Chip[2]^Chip[3];
			  CntH <= 1;
			  PacketType <= DataIn;
			  IncrBufCnt <= 1;
			  BufReady[WritePtr] <= 0;
	        end
      Hdrr: begin
			  PrtyChk <= PrtyChk^DataIn;
			  Header <= {Header[4:0], DataIn};
			  DataWord <= {DataWord[10:0], DataIn};
			  IncrBufCnt <= 0;
              CntH <= CntH + 1;
            end
      Nclu: begin
			  PrtyChk <= PrtyChk^DataIn;
	          DParity <= DParity^DataIn;
              NumClus <= {NumClus[4:0], DataIn};
			  DataWord <= {DataWord[10:0], DataIn};
              CntN <= CntN + 1;
            end
      Regs: begin                              //This section should never execute during normal data acquisition
              CntN <= CntN + 1;
			  DataWord <= {DataWord[10:0], DataIn};
              if (CntN==0) begin
			    Decr2BufCnt <= 1;
//                if (Debug) $display("%g\t DataMon: Chip %h, New register readback.  Header=%b",$time,Chip,Header);
              end else begin
			    Decr2BufCnt <= 0;
//                if (((CntN)%6)==0 && Debug) $display("%g\t DataMon: Chip %h, Register word= h%h  b%b",$time,Chip,DataWord[5:0],DataWord[5:0]);
			  end
            end
      Init: begin
//              if (Debug) $display("%g\t DataMon: New event for chip %h.  Header=%b   NumClus=%b;  %2d Clusters; Tag=%b",$time,Chip,Header,NumClus,NumClus[4:0],Header[3:2]);
              CntC <= 1;
			  PrtyChk <= PrtyChk^DataIn;
			  DataWord <= {DataWord[10:0], DataIn};   //Shifts in just the first bit here
			  Addr <= Addr + 1;
			  if (NumClus == 0) DParity <= 1'b0;
            end
      Shft: begin
			  PrtyChk <= PrtyChk^DataIn;
              DataWord <= {DataWord[10:0], DataIn};
			  DParity <= DParity^DataIn;
//			  IncrBufCnt <= 0;
              if (CntC == 12) begin
//			    if (Debug) $display("%g\t DataMon: Chip %h, Cluster= %2d, %2d",$time,Chip,DataWord[11:6],DataWord[5:0]);
				Addr <= Addr + 1;
				CntC <= 1;
			  end else begin
			    CntC <= CntC + 1;
			  end
            end
      EOEV: begin
//	          IncrBufCnt <= 0;
//              if (NumClus[4:0]>0) begin
//			    if (Debug) $display("%g\t DataMon: Chip %h, Event output completed for tag %b",               $time,Chip,Tag);
//			  end else begin
//			    if (Debug) $display("%g\t DataMon: Chip %h, Event output completed for tag %b; BufCnt=%h",$time,Chip,Tag,BufCnt);
//			  end
//              NEvtOut <= NEvtOut + 1;
			  WritePtr <= WritePtr + 1;
			  ReadyOn <= 1;
			  BufReady[WritePtr] <= 1; 
			  BufParity[WritePtr] <= DParity; 
			  PrtyErr[WritePtr] <= PrtyChk;
			  Done[Tag]<= 1'b1;
            end
    endcase
  end
end

//always @ (posedge EndRun)  //All the $display commands are for debugging, not for the FPGA code
//begin
//  $display("%g\t DataMon: End of simulation for chip %h. Number of events out=%d. Buffer occupancies: %7d %7d %7d %7d %7d %7d",$time,Chip,NEvtOut,NBuf0,NBuf1,NBuf2,NBuf3,NBuf4,NBuf5);
//end

//Instantiate memory for the cluster buffers
assign addra = {WritePtr,Addr};
assign addrb = {ReadPtr,Scntc};
DataMonRAM RAMInstance(Clock, Clock, ena, enb, wea, addra, addrb, DataWord, doa, Word0);

endmodule

//Dual ported RAM with one write port:
module DataMonRAM (clka, clkb, ena, enb, wea, addra, addrb, dia, doa, dob);
input clka, clkb;
input wea;                  //Write enable
input ena, enb;             //Primary and secondary read enables
input [6:0] addra, addrb;   //Addresses for the primary and secondary ports
input [11:0] dia;           //Input data register to write to memory
output [11:0] doa, dob;     //Output registers for the two ports
reg [11:0] RAM [127:0];     //Memory array
reg [11:0] dob, doa;

always @(posedge clka)
begin
	if (ena)
	begin
		if (wea) begin
			RAM[addra]<=dia;  
			doa <= dia;
		end else doa <= RAM[addra];        //Write first, then read
	end
end

always @(posedge clkb)
begin
	if (enb)
	begin
		dob <= RAM[addrb];
	end
end
endmodule