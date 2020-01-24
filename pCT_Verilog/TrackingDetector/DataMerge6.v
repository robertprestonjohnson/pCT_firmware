//Module to merge the six multiple data streams from the pCTFE64 chips
//of the testboard.

module DataMerge6(Clock,Reset,DataIn,EndRun,UnDoBuf,NoRead,MergeDataOut,Address,SendEvt);
input SendEvt;			//1-clock long signal to send the next event
input [3:0] Address;    //Address of the FPGA board (to attach to the header)
input Clock;
input Reset;
input [5:0] DataIn;    //Serial data streams from the N pCTFE64 chips
input EndRun;           //Signal to end the run and output summary information
output [3:0] UnDoBuf;   //Signal to lower the busy flag for the corresponding buffer in the pCTFE64 chips
output NoRead;          //Signal to hold off sending read commands due to DAQ buffers being full
output MergeDataOut;    //Output serial data stream

wire NoRead;
wire [3:0] Address;
reg [3:0] UnDoBuf;
wire FCL0,FCL1,FCL2,FCL3,FCL4,FCL5;
wire [2:0] Cnt0,Cnt1,Cnt2,Cnt3,Cnt4,Cnt5;
reg Send0,Send1,Send2,Send3,Send4,Send5;
wire Clusters0,Clusters1,Clusters2,Clusters3,Clusters4,Clusters5;
wire [1:0] Tag0,Tag1,Tag2,Tag3,Tag4,Tag5;
wire AllGo,AllReady,Ready0,Ready1,Ready2,Ready3,Ready4,Ready5;
wire [5:0] Go;
wire [3:0] Done0,Done1,Done2,Done3,Done4,Done5;
reg [5:0] Reg0Done,Reg1Done,Reg2Done,Reg3Done;
reg [2:0] ThisChip, NextChip;
wire [3:0] NChips;
reg [5:0] Hit;
reg [3:0] CntH;
reg [4:0] CntC;
reg [11:0] Header, ChpHdr;
reg [5:0] ChipDone;          //Flag whether the corresponding chip has been read out
wire [4:0] NClus;
//reg [3:0] i;
wire Parity, Prty0,Prty1,Prty2,Prty3,Prty4,Prty5;
wire TagErr;
reg [5:0] StartString;
reg Stop;
reg [6:0] EvtNum;         //6-bit event counter 

//Define a state machine to coordinate the merged data output
parameter [11:0] Idle = 12'b000000000001;      //Wait for all chips to be ready
parameter [11:0] WtSd = 12'b000000000010;      //Wait for a send event signal
parameter [11:0] Stat = 12'b000000000100;      //Get the status of the chips
parameter [11:0] StBt = 12'b000000001000;      //Output the 6-bit start string
parameter [11:0] Hdrr = 12'b000000010000;      //Output the event header
parameter [11:0] Send = 12'b000000100000;      //Initiate data flow from chip
parameter [11:0] UnSn = 12'b000001000000;      //Terminate the send pulse
parameter [11:0] Kill = 12'b000010000000;      //Kill time for 1 cycle
parameter [11:0] EvHd = 12'b000100000000;      //Decode and output the chip header
parameter [11:0] Chps = 12'b001000000000;      //Output the chip cluster data
parameter [11:0] Wait = 12'b010000000000;      //Wait for the chip to finish
parameter [11:0] CRCD = 12'b100000000000;      //Wait for the CRC calculation to finish
reg [11:0] State, NextState;

//Debug printout, code not to be loaded into the FPGA
//initial begin
//   $display("Time    State DataIn Header    CntH CntC Hit  ThisChip NextChip NChips MergeData UnDoBuf AllRdy Done  Send  Rg0Don FCL0     ChpHdr  Ready  NClus  FCL   Data    Tags    Cnt NoRead Go    StrtStrng DoneCRC MrgeDatOut Stp ChDone");
//end

//always @ (posedge Clock) begin
//   $display("%g\t %b     %b %b %h %h     %b   %h         %h        %h      %b       %b    %b %h%h%h%h%h%h %b%b%b%b%b%b %b %b %b %b%b%b%b%b%b  %h  %h%h%h%h%h%h %b%b%b%b%b%b%b %h%h%h%h%h%h %h%h%h%h%h%h   %b    %b   %b   %b    %b %b %b",          $time,State,DataIn,Header,CntH,CntC,Hit,ThisChip,NextChip,NChips,MergeData,UnDoBuf,AllReady,Done5,Done4,Done3,Done2,Done1,Done0,		         Send5,Send4,Send3,Send2,Send1,Send0,Reg0Done,FCL0,ChpHdr,	         Ready5,Ready4,Ready3,Ready2,Ready1,Ready0,NClus,FCL5,FCL4,FCL3,FCL2,FCL1,FCL0,Clusters5,Clusters4,Clusters3,Clusters2,Clusters1,Clusters0,Header[11],Tag5,Tag4,Tag3,Tag2,Tag1,Tag0,Cnt5,Cnt4,Cnt3,Cnt2,Cnt1,Cnt0,NoRead,Go,StartString,DoneCRC,MergeDataOut,Stop,ChipDone);
//end

CRC6 CRC6_U(MergeDataOut,DoneCRC,MergeData,Stop,Reset,Clock);    //Attach 6-bit CRC to the data stream

assign TagErr = (((Tag0^Tag1) || (Tag0^Tag2) || (Tag0^Tag3) || (Tag0^Tag4) || (Tag0^Tag5))!=2'b00);  //Compare all the trigger tags

//Keep count of how many send event requests have come in
reg [3:0] SendEvtPending;
always @ (posedge Clock) begin
	if (Reset) begin
		SendEvtPending <= 0;
	end else begin
		if (SendEvt && !DoneCRC) begin
			$display("%g\t DataMerge %d: SendEvt signal received, pending = %d",$time,Address,SendEvtPending);
			SendEvtPending <= SendEvtPending + 1;
		end 
		if (DoneCRC && !SendEvt) begin
			$display("%g\t DataMerge %d: DoneCRC signal received, pending = %d",$time,Address,SendEvtPending);
			SendEvtPending <= SendEvtPending - 1;
		end
	end
end

//All of the timing in this state machine is set up to merge the data streams of the
//chips one after the other, with no breaks in between.  Chips with no data do not
//contribute to the stream (not even a header), to minimize data volume and readout time.
always @ (State or AllReady or CntH or CntC or NClus or NextChip or ThisChip or AllGo or DoneCRC or SendEvtPending)
begin
  case (State)
    Idle: begin
				Stop = 1'b0;
				if (AllReady & AllGo)
					if (SendEvtPending > 0)
						NextState = Stat;
					else
						NextState = WtSd;
				else
					NextState = Idle;
			end
	WtSd: begin
			Stop = 1'b0;
			if (SendEvtPending > 0) NextState = Stat;
			else NextState = WtSd;
		  end
	Stat: begin NextState = StBt; Stop = 1'b0; end
	StBt: begin
				Stop = 1'b0;
				if (CntH == 5) NextState = Hdrr;
				else NextState = StBt;
			end
	Hdrr: begin
				Stop = 1'b0;
				if (CntH == 14)      //Shift out all but 2 bits of the header before signaling the first chip to start sending data
					NextState = Send;
				else
					NextState = Hdrr;
			end
	Send: begin NextState = UnSn; Stop = 1'b0; end
	UnSn: begin NextState = Kill; Stop = 1'b0; end
	Kill: begin
				if (NextChip == 6) begin
					NextState = CRCD;
					Stop = 1'b1;
				end else begin
					Stop = 1'b0;
					NextState = EvHd;
				end
			end
	EvHd: begin
				Stop = 1'b0;
				if (CntH == 10)
					NextState = Chps;
				else
					NextState = EvHd;
			 end
	Chps: begin NextState = Wait; Stop = 1'b0; end
	Wait: begin
				Stop = 1'b0;
				if (CntC == NClus && CntH == 8) begin
					NextState = Send;
				end else begin
					NextState = Wait; 
				end
			end
	CRCD: begin
				Stop = 1'b0;
				if (DoneCRC) NextState = Idle;
				else NextState = CRCD;
			end 
	default: begin NextState= Idle; Stop = 1'b0; end
  endcase
end

always @ (posedge Clock)
begin
  if (Reset) begin
    State <= Idle;
	EvtNum <= 0;
	Send0 <= 0;
	Send1 <= 0;
	Send2 <= 0;
	Send3 <= 0;
	Send4 <= 0;
	Send5 <= 0;
  end else begin
    State <= NextState;
	case (State)
	  Idle:  begin
			   Header <= {Header[10:0],1'b0};  //just to flush shift register
			   ChipDone <= 0;
	  		   ThisChip <= 4'b0000;
//			   Stop <= 1'b0;
	         end
	  Stat:  begin
	           CntH <= 0;
			   StartString <= {2'b10,Address};
	  	       Hit[0] <= FCL0;
			   Hit[1] <= FCL1;
			   Hit[2] <= FCL2;
			   Hit[3] <= FCL3;
			   Hit[4] <= FCL4;
			   Hit[5] <= FCL5;   
			   EvtNum <= EvtNum + 1;
//			   if (TagErr)    $display("%g\t MergeData: Tag error.  Tags=%h%h%h%h%h%h",$time,Tag5,Tag4,Tag3,Tag2,Tag1,Tag0);
	         end
	  StBt:  begin
				CntH <= CntH + 1;
				StartString <= {StartString[4:0],1'b0};
				if (CntH == 5) begin
					Header <= {EvtNum[6:2],Tag0,TagErr,NChips};  	
				end
			 end
	  Hdrr:  begin
  			   Header <= {Header[10:0],1'b0};
	           CntH <= CntH + 1;
			 end
	  Send:  begin
	           Header <= {Header[10:0],1'b0};  //Still a couple of bits of the event header to shift out
			   if (ThisChip == 0) Send0 <= 1'b1;
			   if (ThisChip <= 1 && NextChip >= 1) Send1 <= 1'b1;   //Give a send signal to all of the chips
			   if (ThisChip <= 2 && NextChip >= 2) Send2 <= 1'b1;   //in the range.  The ones with no data
			   if (ThisChip <= 3 && NextChip >= 3) Send3 <= 1'b1;   //will not send anything, but they will need
			   if (ThisChip <= 4 && NextChip >= 4) Send4 <= 1'b1;   //to update their buffer pointers.
			   if (ThisChip <= 5 && NextChip >= 5) Send5 <= 1'b1;
			 end
	  UnSn:  begin
	           Header <= {Header[10:0],1'b0};  //Still another bit of the event header to shift out
	           Send0 <= 1'b0;
			   Send1 <= 1'b0;
			   Send2 <= 1'b0;
			   Send3 <= 1'b0;
			   Send4 <= 1'b0;
			   Send5 <= 1'b0;	 		   
               case (NextChip)
			     4'h0: ChipDone[0] <= 1'b1;
			     4'h1: ChipDone[1] <= 1'b1;
			     4'h2: ChipDone[2] <= 1'b1;
			     4'h3: ChipDone[3] <= 1'b1;
			     4'h4: ChipDone[4] <= 1'b1;
			     4'h5: ChipDone[5] <= 1'b1;
// 				  4'h6: Stop <= 1'b1;
               endcase			 
	         end	
      Kill:  begin
//			   Stop <= 1'b0;
//				if (NextChip == 6) Stop <= 1'b1;
	           CntH <= 0;
			   ThisChip <= NextChip + 1;	
	           Header <= {Header[10:0],1'b0};  //Shift once more to make sure that a 0 stays on this net			   
             end	  
	  EvHd:  begin
	           CntH <= CntH + 1;
			   ChpHdr <= {ChpHdr[10:0],MergeData};  //shift in all of the chip header
			 end
	  Chps:  begin
               ChpHdr <= {ChpHdr[10:0],MergeData};  //Get the last bit of the chip header
			   CntH <= 0;
			   CntC <= 1;
	         end
	  Wait:  begin                       //Dead reckon when to start the data flowing from the next chip
               if (CntH == 11) begin
						CntH <= 0;
						CntC <= CntC + 1;
					end else begin
						CntH <= CntH + 1;
					end
	         end
//		CRCD: Stop <= 1'b0;
	endcase
  end
end

assign NClus = ChpHdr[10:6];

DataMon DataMon0(4'b0000,Clock,Reset,DataIn[0],EndRun,Send0,Done0,Ready0,FCL0,Tag0,Clusters0,Cnt0,Go[0],Prty0);
DataMon DataMon1(4'b0001,Clock,Reset,DataIn[1],EndRun,Send1,Done1,Ready1,FCL1,Tag1,Clusters1,Cnt1,Go[1],Prty1);
DataMon DataMon2(4'b0010,Clock,Reset,DataIn[2],EndRun,Send2,Done2,Ready2,FCL2,Tag2,Clusters2,Cnt2,Go[2],Prty2);
DataMon DataMon3(4'b0011,Clock,Reset,DataIn[3],EndRun,Send3,Done3,Ready3,FCL3,Tag3,Clusters3,Cnt3,Go[3],Prty3);
DataMon DataMon4(4'b0100,Clock,Reset,DataIn[4],EndRun,Send4,Done4,Ready4,FCL4,Tag4,Clusters4,Cnt4,Go[4],Prty4);
DataMon DataMon5(4'b0101,Clock,Reset,DataIn[5],EndRun,Send5,Done5,Ready5,FCL5,Tag5,Clusters5,Cnt5,Go[5],Prty5);

assign AllReady = Ready0 & Ready1 & Ready2 & Ready3 & Ready4 & Ready5;
assign MergeData = Clusters0 | Clusters1 | Clusters2 | Clusters3 | Clusters4 | Clusters5 | Header[11] | StartString[5];
assign 	NChips = Hit[0] + Hit[1] + Hit[2] + Hit[3] + Hit[4] + Hit[5];
assign NoRead = (Cnt0>=5) || (Cnt1>=5) || (Cnt2>=5) || (Cnt3>=5) || (Cnt4>=5) || (Cnt5>=5);
assign AllGo = Go[0] & Go[1] & Go[2] & Go[3] & Go[4] & Go[5];
assign Parity = Prty0^Prty1^Prty2^Prty3^Prty4^Prty5^Tag0[0]^Tag0[1]^NChips[0]^NChips[1]^NChips[2]^NChips[3];

//Keep track of which is the next chip with clusters & number of such chips
always @ (posedge Clock)
begin
  if (Reset) begin
    NextChip <= 6;
  end else begin
    if (Hit[0] & !ChipDone[0])
	  NextChip <= 0;
	else if (Hit[1] & !ChipDone[1])
	  NextChip <= 1;
	else if (Hit[2] & !ChipDone[2])
	  NextChip <= 2;
	else if (Hit[3] & !ChipDone[3])
	  NextChip <= 3;
	else if (Hit[4] & !ChipDone[4])
	  NextChip <= 4;
	else if (Hit[5] & !ChipDone[5])
	  NextChip <= 5;
	else
	  NextChip <= 6;
  end
end

//Signal for each buffer when all chips have finished outputing the corresponding data
always @ (posedge Clock)
begin
  if (Reset) begin
    Reg0Done <= 0;
	Reg1Done <= 0;
	Reg2Done <= 0;
	Reg3Done <= 0;
	UnDoBuf <= 4'b0000;
  end else begin
    if (Reg0Done == 6'b111111) begin
	  UnDoBuf[0] <= 1;
	  Reg0Done <= 0;
	end else begin
	  if (Done0[0]) Reg0Done[0] <= 1;
	  if (Done1[0]) Reg0Done[1] <= 1;
	  if (Done2[0]) Reg0Done[2] <= 1;
	  if (Done3[0]) Reg0Done[3] <= 1;
	  if (Done4[0]) Reg0Done[4] <= 1;
	  if (Done5[0]) Reg0Done[5] <= 1;
	  UnDoBuf[0] <= 0;
	end
	if (Reg1Done == 6'b111111) begin
	  UnDoBuf[1] <= 1;
	  Reg1Done <= 0;
	end else begin
	  if (Done0[1]) Reg1Done[0] <= 1;
	  if (Done1[1]) Reg1Done[1] <= 1;
	  if (Done2[1]) Reg1Done[2] <= 1;
	  if (Done3[1]) Reg1Done[3] <= 1;
	  if (Done4[1]) Reg1Done[4] <= 1;
	  if (Done5[1]) Reg1Done[5] <= 1;
	  UnDoBuf[1] <= 0;
    end
	if (Reg2Done == 6'b111111) begin
	  UnDoBuf[2] <= 1;
	  Reg2Done <= 0;
	end else begin
	  if (Done0[2]) Reg2Done[0] <= 1;
	  if (Done1[2]) Reg2Done[1] <= 1;
	  if (Done2[2]) Reg2Done[2] <= 1;
	  if (Done3[2]) Reg2Done[3] <= 1;
	  if (Done4[2]) Reg2Done[4] <= 1;
	  if (Done5[2]) Reg2Done[5] <= 1;
	  UnDoBuf[2] <= 0;
    end
	if (Reg3Done == 6'b111111) begin
	    UnDoBuf[3] <= 1;
	    Reg3Done <= 0;
	end else begin
	  if (Done0[3]) Reg3Done[0] <= 1;
	  if (Done1[3]) Reg3Done[1] <= 1;
	  if (Done2[3]) Reg3Done[2] <= 1;
	  if (Done3[3]) Reg3Done[3] <= 1;
	  if (Done4[3]) Reg3Done[4] <= 1;
	  if (Done5[3]) Reg3Done[5] <= 1;
	  UnDoBuf[3] <= 0;
    end
  end
end

endmodule

