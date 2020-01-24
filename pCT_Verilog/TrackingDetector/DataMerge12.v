//Module to merge the twelve multiple data streams from the 12 pCTFE64 chips
//of the tracker V board, or half of a T board.
//R. Johnson
//July 28, 2013, add chip mask
//August 14, 2013, added Address to CRC6 arguments, for debugging only
//October 3, 2013, synch the event number with the trigger tag
//March 7, 2014, add diagnostics for detected errors

module DataMerge12(Clock,Reset,DataIn,EndRun,UnDoBuf,NoRead,MergeDataOut,Address,SendEvt,Mask,ErrBuf,Nevent);
input [11:0] Mask;		//Selects which chips are connected and working and are to be used
input SendEvt;			//1-clock long signal to send the next event
input [3:0] Address;    //Address of the FPGA board (to attach to the header)
input Clock;
input Reset;
input [11:0] DataIn;    //Serial data streams from the N pCTFE64 chips
input EndRun;           //Signal to end the run and output summary information
output [3:0] UnDoBuf;   //Signal to lower the busy flag for the corresponding buffer in the pCTFE64 chips
output NoRead;          //Signal to hold off sending read commands due to DAQ buffers being full
output MergeDataOut;    //Output serial data stream
output [47:0] ErrBuf;	//Register information for debugging in case of errors
output [15:0] Nevent;

wire NoRead;
wire [3:0] Address;
reg [3:0] UnDoBuf;
wire [11:0] FCL;
wire [2:0] Cnt0,Cnt1,Cnt2,Cnt3,Cnt4,Cnt5,Cnt6,Cnt7,Cnt8,Cnt9,CntA,CntB;
reg [11:0] Send;
wire [11:0] Clusters;
wire [1:0] Tag0,Tag1,Tag2,Tag3,Tag4,Tag5,Tag6,Tag7,Tag8,Tag9,TagA,TagB;
wire AllGo, AllReady;
wire [11:0] Ready;
wire [11:0] Go;
wire [3:0] Done0,Done1,Done2,Done3,Done4,Done5,Done6,Done7,Done8,Done9,DoneA,DoneB;
reg [11:0] Reg0Done,Reg1Done,Reg2Done,Reg3Done;
reg [3:0] ThisChip, NextChip;
wire [3:0] NChips;
reg [11:0] Hit;
reg [3:0] CntH;
reg [4:0] CntC;
reg [11:0] Header, ChpHdr;
reg [11:0] ChipDone;          //Flag whether the corresponding chip has been read out
wire [4:0] NClus;
//reg [3:0] i;
wire Parity;
wire [11:0] Prty;
wire TagErr;
reg [47:0] ErrBuf;
reg [5:0] StartString;
reg Stop;
reg [4:0] EvtNum;         //5-bit event counter 

//Define a state machine to coordinate the merged data output
parameter [11:0] Idle = 12'b000000000001;      //Wait for all chips to be ready
parameter [11:0] WtSd = 12'b000000000010;      //Wait for a send event signal
parameter [11:0] Stat = 12'b000000000100;      //Get the status of the chips
parameter [11:0] StBt = 12'b000000001000;      //Output the 6-bit start string
parameter [11:0] Hdrr = 12'b000000010000;      //Output the event header
parameter [11:0] SndI = 12'b000000100000;      //Initiate data flow from chip
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
//   $display("%g\t %b     %b %b %h %h     %b   %h         %h        %h      %b       %b    %b %h%h%h%h%h%h %b %b %b %b %b  %h  %b %b %b %h%h%h%h%h%h %h%h%h%h%h%h   %b    %b   %b   %b    %b %b %b",          $time,State,DataIn,Header,CntH,CntC,Hit,ThisChip,NextChip,NChips,MergeData,UnDoBuf,AllReady,Done5,Done4,Done3,Done2,Done1,Done0,		         Send,Reg0Done,FCL[0],ChpHdr,	         Ready,NClus,FCL,Clusters,Header[11],Tag5,Tag4,Tag3,Tag2,Tag1,Tag0,Cnt5,Cnt4,Cnt3,Cnt2,Cnt1,Cnt0,NoRead,Go,StartString,DoneCRC,MergeDataOut,Stop,ChipDone);
//end

CRC6 CRC6_U(MergeDataOut,DoneCRC,MergeData,Stop,Reset,Clock,Address);    //Attach 6-bit CRC to the data stream

assign TagErr = (((Tag0^Tag1) || (Tag0^Tag2) || (Tag0^Tag3) || (Tag0^Tag4) || (Tag0^Tag5) || (Tag0^Tag6) || (Tag0^Tag7) || (Tag0^Tag8) || (Tag0^Tag9) || (Tag0^TagA) || (Tag0^TagB)) != 2'b00);  //Compare all the trigger tags

//Keep count of how many send event requests have come in
reg [3:0] SendEvtPending;
always @ (posedge Clock) begin
	if (Reset) begin
		SendEvtPending <= 0;
	end else begin
		if (SendEvt && !DoneCRC) begin
//			$display("%g\t DataMerge %d: SendEvt signal received, pending = %d",$time,Address,SendEvtPending);
			SendEvtPending <= SendEvtPending + 1;
		end 
		if (DoneCRC && !SendEvt) begin
//			$display("%g\t DataMerge %d: DoneCRC signal received, pending = %d",$time,Address,SendEvtPending);
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
					NextState = SndI;
				else
					NextState = Hdrr;
			end
	SndI: begin NextState = UnSn; Stop = 1'b0; end
	UnSn: begin NextState = Kill; Stop = 1'b0; end
	Kill: begin
				if (NextChip == 12) begin
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
					NextState = SndI;
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

reg [15:0] Nevent;
always @ (posedge Clock)
begin
  if (Reset) begin
    State <= Idle;
	EvtNum <= 0;
	Send <= 0;
	ErrBuf[47:24] <= 0;
	Nevent <= 0;
  end else begin
    State <= NextState;
//	if (State != Idle && Address==4'b0000) $display("%g\t DataMerge12: State=%b, MergeData=%b, MergeDataOut=%b, DataIn=%b, Clusters=%b, Hdr=%b, Strt=%b",$time,State,MergeData,MergeDataOut,DataIn,Clusters,Header,StartString);
	case (State)
	  Idle:  begin
			   Header <= {Header[10:0],1'b0};  //just to flush shift register
			   StartString <= 6'b000000;
			   ChipDone <= 0;
	  		   ThisChip <= 4'b0000;
	         end
	  Stat:  begin
					Nevent <= Nevent + 1;
	           CntH <= 0;
			   StartString <= {2'b10,Address};
			   Hit <= FCL & Mask;
			   if (TagErr) ErrBuf[47:24] <= {TagB,TagA,Tag9,Tag8,Tag7,Tag6,Tag5,Tag4,Tag3,Tag2,Tag1};
	         end
	  StBt:  begin
				CntH <= CntH + 1;
				StartString <= {StartString[4:0],1'b0};
				if (CntH == 5) begin
					Header <= {EvtNum,Tag0,TagErr,NChips}; 
					if (Tag0 == 2'b11) EvtNum <= EvtNum + 1;
				end
			 end
	  Hdrr:  begin
  			   Header <= {Header[10:0],1'b0};
	           CntH <= CntH + 1;
			 end
	  SndI:  begin
	           Header <= {Header[10:0],1'b0};  //Still a couple of bits of the event header to shift out
			   if (ThisChip == 0) Send[0] <= 1'b1;
			   if (ThisChip <= 1 && NextChip >= 1) Send[1] <= 1'b1;   //Give a send signal to all of the chips
			   if (ThisChip <= 2 && NextChip >= 2) Send[2] <= 1'b1;   //in the range.  The ones with no data
			   if (ThisChip <= 3 && NextChip >= 3) Send[3] <= 1'b1;   //will not send anything, but they will need
			   if (ThisChip <= 4 && NextChip >= 4) Send[4] <= 1'b1;   //to update their buffer pointers.
			   if (ThisChip <= 5 && NextChip >= 5) Send[5] <= 1'b1;
			   if (ThisChip <= 6 && NextChip >= 6) Send[6] <= 1'b1;   
			   if (ThisChip <= 7 && NextChip >= 7) Send[7] <= 1'b1;   
			   if (ThisChip <= 8 && NextChip >= 8) Send[8] <= 1'b1;   
			   if (ThisChip <= 9 && NextChip >= 9) Send[9] <= 1'b1;   
			   if (ThisChip <= 10 && NextChip >= 10) Send[10] <= 1'b1;
			   if (ThisChip <= 11 && NextChip >= 11) Send[11] <= 1'b1;
			 end
	  UnSn:  begin
	           Header <= {Header[10:0],1'b0};  //Still another bit of the event header to shift out
			   Send <= 0;		   
			   ChipDone[NextChip] <= 1'b1;
	         end	
      Kill:  begin
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
	endcase
  end
end

assign NClus = ChpHdr[10:6];

wire [11:0] PrtyOut;
DataMon DataMon0(4'b0000,Clock,Reset,DataIn[0],EndRun,Send[0],Done0,Ready[0],FCL[0],Tag0,Clusters[0],Cnt0,Go[0],PrtyOut[0]);
DataMon DataMon1(4'b0001,Clock,Reset,DataIn[1],EndRun,Send[1],Done1,Ready[1],FCL[1],Tag1,Clusters[1],Cnt1,Go[1],PrtyOut[1]);
DataMon DataMon2(4'b0010,Clock,Reset,DataIn[2],EndRun,Send[2],Done2,Ready[2],FCL[2],Tag2,Clusters[2],Cnt2,Go[2],PrtyOut[2]);
DataMon DataMon3(4'b0011,Clock,Reset,DataIn[3],EndRun,Send[3],Done3,Ready[3],FCL[3],Tag3,Clusters[3],Cnt3,Go[3],PrtyOut[3]);
DataMon DataMon4(4'b0100,Clock,Reset,DataIn[4],EndRun,Send[4],Done4,Ready[4],FCL[4],Tag4,Clusters[4],Cnt4,Go[4],PrtyOut[4]);
DataMon DataMon5(4'b0101,Clock,Reset,DataIn[5],EndRun,Send[5],Done5,Ready[5],FCL[5],Tag5,Clusters[5],Cnt5,Go[5],PrtyOut[5]);
DataMon DataMon6(4'b0110,Clock,Reset,DataIn[6],EndRun,Send[6],Done6,Ready[6],FCL[6],Tag6,Clusters[6],Cnt6,Go[6],PrtyOut[6]);
DataMon DataMon7(4'b0111,Clock,Reset,DataIn[7],EndRun,Send[7],Done7,Ready[7],FCL[7],Tag7,Clusters[7],Cnt7,Go[7],PrtyOut[7]);
DataMon DataMon8(4'b1000,Clock,Reset,DataIn[8],EndRun,Send[8],Done8,Ready[8],FCL[8],Tag8,Clusters[8],Cnt8,Go[8],PrtyOut[8]);
DataMon DataMon9(4'b1001,Clock,Reset,DataIn[9],EndRun,Send[9],Done9,Ready[9],FCL[9],Tag9,Clusters[9],Cnt9,Go[9],PrtyOut[9]);
DataMon DataMonA(4'b1010,Clock,Reset,DataIn[10],EndRun,Send[10],DoneA,Ready[10],FCL[10],TagA,Clusters[10],CntA,Go[10],PrtyOut[10]);
DataMon DataMonB(4'b1011,Clock,Reset,DataIn[11],EndRun,Send[11],DoneB,Ready[11],FCL[11],TagB,Clusters[11],CntB,Go[11],PrtyOut[11]);

//State machine to monitor the Ready flags
parameter [3:0] WaitRd = 4'b0001;
parameter [3:0] LookRd = 4'b0010;
parameter [3:0] ErrrRd = 4'b0100;
parameter [3:0] DoneRd = 4'b1000;

reg [3:0] StateRd, NextStateRd;
reg [7:0] CntRd;
always @ (StateRd or Ready or AllReady or Mask or CntRd) begin
	case (StateRd)
		WaitRd: begin
					if ((Ready & Mask) != 0) NextStateRd = LookRd;
					else NextStateRd = WaitRd;
				end
		LookRd:	begin
					if (AllReady) NextStateRd = DoneRd;
					else if (CntRd == 8'b11111111) NextStateRd = ErrrRd;
					else NextStateRd = LookRd;
				end
		ErrrRd:	begin
					NextStateRd = DoneRd;
				end
		DoneRd:	begin
					if ((Ready & Mask) == 0) NextStateRd = WaitRd;
					else NextStateRd = DoneRd;
				end
		default:	begin
						NextStateRd = WaitRd;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		StateRd <= WaitRd;
		ErrBuf[11:0] <= 0;
	end else begin
		StateRd <= NextStateRd;
		case (StateRd)
			WaitRd:	begin
						CntRd <= 0;
					end
			LookRd:	begin
//						if (Address == 0) $display("%g\t DataMerge %d, State LookRd, Ready=%b, CntRd=%d.",$time,Address,Ready,CntRd);
						CntRd <= CntRd + 1;
					end
			ErrrRd:	begin
						$display("%g\t DataMerge %d, State ErrrRd, Ready=%b",$time,Address,Ready);
						ErrBuf[11:0] <= Ready | ~Mask;  //Save the ready status if we've been waiting 256 clock cycles
					end
		endcase
	end
end

//State machine to monitor the Go flags
parameter [3:0] WaitGo = 4'b0001;
parameter [3:0] LookGo = 4'b0010;
parameter [3:0] ErrrGo = 4'b0100;
parameter [3:0] DoneGo = 4'b1000;

reg [3:0] StateGo, NextStateGo;
reg [7:0] CntGo;
always @ (StateGo or AllGo or Go or Mask or CntGo) begin
	case (StateGo)
		WaitGo: begin
					if ((Go & Mask) != 0) NextStateGo = LookGo;
					else NextStateGo = WaitGo;
				end
		LookGo:	begin
					if (AllGo) NextStateGo = DoneGo;
					else if (CntGo == 8'b11111111) NextStateGo = ErrrGo;
					else NextStateGo = LookGo;
				end
		ErrrGo:	begin
					NextStateGo = DoneGo;
				end
		DoneGo: begin
					if ((Go & Mask) == 0) NextStateGo = WaitGo;
					else NextStateGo = DoneGo;
				end
		default:	begin
						NextStateGo = WaitGo;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		StateGo <= WaitGo;
		ErrBuf[23:12] <= 0;
	end else begin
		StateGo <= NextStateGo;
		case (StateGo)
			WaitGo:	begin
						CntGo <= 0;
					end
			LookGo:	begin
						//if (Address == 0) $display("%g\t DataMerge %d, State LookGo, Go=%b, CntGo=%d.",$time,Address,Go,CntGo);
						CntGo <= CntGo + 1;
					end
			ErrrGo:	begin
						$display("%g\t DataMerge %d, State ErrrGo, Go=%b",$time,Address,Go);
						ErrBuf[23:12] <= Go | ~Mask;  //Save the Go status if we've been waiting 256 clock cycles
					end
		endcase
	end
end

assign Prty = (PrtyOut & Mask);
assign AllReady = ((Ready | ~Mask) == 12'b111111111111);
assign MergeData = Clusters[0] | Clusters[1] | Clusters[2] | Clusters[3] | Clusters[4] | Clusters[5] | Clusters[6] | Clusters[7] | Clusters[8] | Clusters[9] | Clusters[10] | Clusters[11] | Header[11] | StartString[5];
assign 	NChips = Hit[0] + Hit[1] + Hit[2] + Hit[3] + Hit[4] + Hit[5] + Hit[6] + Hit[7] + Hit[8] + Hit[9] + Hit[10] + Hit[11];
assign NoRead = (Cnt0>=5) || (Cnt1>=5) || (Cnt2>=5) || (Cnt3>=5) || (Cnt4>=5) || (Cnt5>=5) || (Cnt6>=5) || (Cnt7>=5) || (Cnt8>=5) || (Cnt9>=5) || (CntA>=5) || (CntB>=5);
assign AllGo = ((Go | ~Mask) == 12'b111111111111);
assign Parity = Prty[0]^Prty[1]^Prty[2]^Prty[3]^Prty[4]^Prty[5]^Prty[6]^Prty[7]^Prty[8]^Prty[9]^Prty[10]^Prty[11]^Tag0[0]^Tag0[1]^NChips[0]^NChips[1]^NChips[2]^NChips[3];

//Keep track of which is the next chip with clusters & number of such chips
wire [11:0] ChipDoneM;
assign ChipDoneM = ChipDone | ~Mask;
always @ (posedge Clock)
begin
  if (Reset) begin
    NextChip <= 12;
  end else begin
    if (Hit[0] & !ChipDoneM[0])
	  NextChip <= 0;
	else if (Hit[1] & !ChipDoneM[1])
	  NextChip <= 1;
	else if (Hit[2] & !ChipDoneM[2])
	  NextChip <= 2;
	else if (Hit[3] & !ChipDoneM[3])
	  NextChip <= 3;
	else if (Hit[4] & !ChipDoneM[4])
	  NextChip <= 4;
	else if (Hit[5] & !ChipDoneM[5])
	  NextChip <= 5;
	else if (Hit[6] & !ChipDoneM[6])
	  NextChip <= 6;
	else if (Hit[7] & !ChipDoneM[7])
	  NextChip <= 7;
	else if (Hit[8] & !ChipDoneM[8])
	  NextChip <= 8;
	else if (Hit[9] & !ChipDoneM[9])
	  NextChip <= 9;
	else if (Hit[10] & !ChipDoneM[10])
	  NextChip <= 10;
	else if (Hit[11] & !ChipDoneM[11])
	  NextChip <= 11;
	else
	  NextChip <= 12;
  end
end

////Keep track of which is the next chip with clusters & number of such chips
// wire [11:0] ChipDoneM, HitNotDone;
// reg [11:0] NextChipA;
// assign ChipDoneM = ChipDone | ~Mask;
// assign HitNotDone = Hit & ~ChipDoneM;
////Priority encoder:
// always @ (HitNotDone) begin
	// if 		(HitNotDone[0]) NextChipA = 0;
	// else if (HitNotDone[1]) NextChipA = 1;
	// else if (HitNotDone[2]) NextChipA = 2;
	// else if (HitNotDone[3]) NextChipA = 3;
	// else if (HitNotDone[4]) NextChipA = 4;
	// else if (HitNotDone[5]) NextChipA = 5;
	// else if (HitNotDone[6]) NextChipA = 6;
	// else if (HitNotDone[7]) NextChipA = 7;
	// else if (HitNotDone[8]) NextChipA = 8;
	// else if (HitNotDone[9]) NextChipA = 9;
	// else if (HitNotDone[10]) NextChipA = 10;
	// else if (HitNotDone[11]) NextChipA = 11;
	// else					 NextChipA = 12;
// end

// always @ (posedge Clock) begin
	// if (Reset) begin
		// NextChip <= 12;
	// end else begin
		// NextChip <= NextChipA;
	// end
// end

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
    if (Reg0Done == 12'b111111111111) begin
	  UnDoBuf[0] <= 1;
	  Reg0Done <= 0;
	end else begin
	  if (Done0[0]) Reg0Done[0] <= 1;
	  if (Done1[0]) Reg0Done[1] <= 1;
	  if (Done2[0]) Reg0Done[2] <= 1;
	  if (Done3[0]) Reg0Done[3] <= 1;
	  if (Done4[0]) Reg0Done[4] <= 1;
	  if (Done5[0]) Reg0Done[5] <= 1;
	  if (Done6[0]) Reg0Done[6] <= 1;
	  if (Done7[0]) Reg0Done[7] <= 1;
	  if (Done8[0]) Reg0Done[8] <= 1;
	  if (Done9[0]) Reg0Done[9] <= 1;
	  if (DoneA[0]) Reg0Done[10] <= 1;
	  if (DoneB[0]) Reg0Done[11] <= 1;
	  UnDoBuf[0] <= 0;
	end
	if (Reg1Done == 12'b111111111111) begin
	  UnDoBuf[1] <= 1;
	  Reg1Done <= 0;
	end else begin
	  if (Done0[1]) Reg1Done[0] <= 1;
	  if (Done1[1]) Reg1Done[1] <= 1;
	  if (Done2[1]) Reg1Done[2] <= 1;
	  if (Done3[1]) Reg1Done[3] <= 1;
	  if (Done4[1]) Reg1Done[4] <= 1;
	  if (Done5[1]) Reg1Done[5] <= 1;
	  if (Done6[1]) Reg1Done[6] <= 1;
	  if (Done7[1]) Reg1Done[7] <= 1;
	  if (Done8[1]) Reg1Done[8] <= 1;
	  if (Done9[1]) Reg1Done[9] <= 1;
	  if (DoneA[1]) Reg1Done[10] <= 1;
	  if (DoneB[1]) Reg1Done[11] <= 1;
	  UnDoBuf[1] <= 0;
    end
	if (Reg2Done == 12'b111111111111) begin
	  UnDoBuf[2] <= 1;
	  Reg2Done <= 0;
	end else begin
	  if (Done0[2]) Reg2Done[0] <= 1;
	  if (Done1[2]) Reg2Done[1] <= 1;
	  if (Done2[2]) Reg2Done[2] <= 1;
	  if (Done3[2]) Reg2Done[3] <= 1;
	  if (Done4[2]) Reg2Done[4] <= 1;
	  if (Done5[2]) Reg2Done[5] <= 1;
	  if (Done6[2]) Reg2Done[6] <= 1;
	  if (Done7[2]) Reg2Done[7] <= 1;
	  if (Done8[2]) Reg2Done[8] <= 1;
	  if (Done9[2]) Reg2Done[9] <= 1;
	  if (DoneA[2]) Reg2Done[10] <= 1;
	  if (DoneB[2]) Reg2Done[11] <= 1;
	  UnDoBuf[2] <= 0;
    end
	if (Reg3Done == 12'b111111111111) begin
	    UnDoBuf[3] <= 1;
	    Reg3Done <= 0;
	end else begin
	  if (Done0[3]) Reg3Done[0] <= 1;
	  if (Done1[3]) Reg3Done[1] <= 1;
	  if (Done2[3]) Reg3Done[2] <= 1;
	  if (Done3[3]) Reg3Done[3] <= 1;
	  if (Done4[3]) Reg3Done[4] <= 1;
	  if (Done5[3]) Reg3Done[5] <= 1;
	  if (Done6[3]) Reg3Done[6] <= 1;
	  if (Done7[3]) Reg3Done[7] <= 1;
	  if (Done8[3]) Reg3Done[8] <= 1;
	  if (Done9[3]) Reg3Done[9] <= 1;
	  if (DoneA[3]) Reg3Done[10] <= 1;
	  if (DoneB[3]) Reg3Done[11] <= 1;
	  UnDoBuf[3] <= 0;
    end
  end
end

endmodule

