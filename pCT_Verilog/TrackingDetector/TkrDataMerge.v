//Module to merge the twelve multiple data streams from the 12 pCTFE64 chips
//of the tracker V board, or half of a T board.  This is a completely new version,
//to try to make it more reliable and maintainable.
//R. Johnson
//April 9, 2014
//April 30, 2014, fixed bug that was dropping the last bit of the output stream
//August 19, 2014, improvements in buffer clear signals and ErrBuf

module TkrDataMerge(ErrorOut,Nevent,MergeDataOut,BufClrAll,ErrBuf,Clock,Reset,DataIn,Address,SendEvt,Mask);
input [11:0] Mask;        //Selects which chips are connected and working and are to be used
input SendEvt;            //1-clock long signal to send the next event
input [3:0] Address;    //Address of the FPGA board (to attach to the header)
input Clock;
input Reset;
input [11:0] DataIn;    //Serial data streams from the N pCTFE64 chips, already masked
output BufClrAll;       //Signal that an ASIC buffer was just freed up
output MergeDataOut;    //Output serial data stream
output [47:0] ErrBuf;    //Register information for debugging in case of errors
output [15:0] Nevent;
output ErrorOut;        //One clock signal for each error detected, for monitoring

wire [3:0] Address;
reg BufClrAll;
reg [47:0] ErrBuf;
reg MergeData, Stop;

reg [4:0] EvtNum;         //5-bit event counter 

CRC6 CRC6_U(MergeDataOut,DoneCRC,MergeData,Stop,Reset,Clock,Address);    //Attach 6-bit CRC to the data stream

reg [15:0] Strobe;   //Only 12 chips, but this is overdimensioned so that it can be addressed by a 4-bit word
reg Send;
wire [11:0] Error, DataOut0, DataOut1, DataOut2, DataOut3, DataOut4, DataOut5, DataOut6;
wire [11:0] DataOut7, DataOut8, DataOut9, DataOutA, DataOutB;
wire [3:0] nFull0, nFull1, nFull2, nFull3, nFull4, nFull5, nFull6, nFull7, nFull8, nFull9, nFullA, nFullB;
wire [11:0] BufClr0, BufClr1, BufClr2, BufClr3;

assign ErrorOut = (Error != 0);
//One instance of the following for each ASIC, to receive, parse, and buffer the event data.
TkrDatReceive TkrDatReceive0(BufClr0[0],BufClr1[0],BufClr2[0],BufClr3[0],nFull0,PrtyErr[0],Error[0],DataOut0,Strobe[0],Send,DataIn[0],Clock,Reset,4'h0,Address);
TkrDatReceive TkrDatReceive1(BufClr0[1],BufClr1[1],BufClr2[1],BufClr3[1],nFull1,PrtyErr[1],Error[1],DataOut1,Strobe[1],Send,DataIn[1],Clock,Reset,4'h1,Address);
TkrDatReceive TkrDatReceive2(BufClr0[2],BufClr1[2],BufClr2[2],BufClr3[2],nFull2,PrtyErr[2],Error[2],DataOut2,Strobe[2],Send,DataIn[2],Clock,Reset,4'h2,Address);
TkrDatReceive TkrDatReceive3(BufClr0[3],BufClr1[3],BufClr2[3],BufClr3[3],nFull3,PrtyErr[3],Error[3],DataOut3,Strobe[3],Send,DataIn[3],Clock,Reset,4'h3,Address);
TkrDatReceive TkrDatReceive4(BufClr0[4],BufClr1[4],BufClr2[4],BufClr3[4],nFull4,PrtyErr[4],Error[4],DataOut4,Strobe[4],Send,DataIn[4],Clock,Reset,4'h4,Address);
TkrDatReceive TkrDatReceive5(BufClr0[5],BufClr1[5],BufClr2[5],BufClr3[5],nFull5,PrtyErr[5],Error[5],DataOut5,Strobe[5],Send,DataIn[5],Clock,Reset,4'h5,Address);
TkrDatReceive TkrDatReceive6(BufClr0[6],BufClr1[6],BufClr2[6],BufClr3[6],nFull6,PrtyErr[6],Error[6],DataOut6,Strobe[6],Send,DataIn[6],Clock,Reset,4'h6,Address);
TkrDatReceive TkrDatReceive7(BufClr0[7],BufClr1[7],BufClr2[7],BufClr3[7],nFull7,PrtyErr[7],Error[7],DataOut7,Strobe[7],Send,DataIn[7],Clock,Reset,4'h7,Address);
TkrDatReceive TkrDatReceive8(BufClr0[8],BufClr1[8],BufClr2[8],BufClr3[8],nFull8,PrtyErr[8],Error[8],DataOut8,Strobe[8],Send,DataIn[8],Clock,Reset,4'h8,Address);
TkrDatReceive TkrDatReceive9(BufClr0[9],BufClr1[9],BufClr2[9],BufClr3[9],nFull9,PrtyErr[9],Error[9],DataOut9,Strobe[9],Send,DataIn[9],Clock,Reset,4'h9,Address);
TkrDatReceive TkrDatReceiveA(BufClr0[10],BufClr1[10],BufClr2[10],BufClr3[10],nFullA,PrtyErr[10],Error[10],DataOutA,Strobe[10],Send,DataIn[10],Clock,Reset,4'ha,Address);
TkrDatReceive TkrDatReceiveB(BufClr0[11],BufClr1[11],BufClr2[11],BufClr3[11],nFullB,PrtyErr[11],Error[11],DataOutB,Strobe[11],Send,DataIn[11],Clock,Reset,4'hb,Address);

//Manage the buffer clear signals and report on chips that fail to send data
wire [11:0] ErrBuf0, ErrBuf1, ErrBuf2, ErrBuf3;
BufClrTkr BufClrTkr0(ErrBuf0,BufClrAll0,BufClr0,Mask,Clock,Reset);   //For tag 0
BufClrTkr BufClrTkr1(ErrBuf1,BufClrAll1,BufClr1,Mask,Clock,Reset);   //For tag 1
BufClrTkr BufClrTkr2(ErrBuf2,BufClrAll2,BufClr2,Mask,Clock,Reset);   //For tag 2
BufClrTkr BufClrTkr3(ErrBuf3,BufClrAll3,BufClr3,Mask,Clock,Reset);   //For tag 3

//State machine to send out all the buffer clear signals
parameter [2:0] WaitBc=3'b001;
parameter [2:0] SendBc=3'b010;
parameter [3:0] PausBc=3'b100;
reg [2:0] StateBc, NextStateBc;

always @ (StateBc or NBufClrToDo) begin
    case (StateBc)
        WaitBc:		begin
						if (NBufClrToDo > 0) NextStateBc = SendBc;
						else NextStateBc = WaitBc;
					end
		SendBc:		begin
						NextStateBc = PausBc;
					end
		PausBc:     begin
		                NextStateBc = WaitBc;
					end
		default: 	NextStateBc = WaitBc;
    endcase
end

reg [2:0] NBufClrToDo;
always @ (posedge Clock) begin
    if (Reset) begin
        StateBc <= WaitBc;
		NBufClrToDo <= 0;
		BufClrAll <= 1'b0;
    end else begin
	    if (BufClrAll0 | BufClrAll1 | BufClrAll2 | BufClrAll3) $display("%g\t TkrDataMerge %d: buffer clear pulse: %b%b%b%b",$time,Address,BufClrAll0,BufClrAll1,BufClrAll2,BufClrAll3);
		ErrBuf[23:12] <= ErrBuf0 | ErrBuf1 | ErrBuf2 | ErrBuf3;
        StateBc <= NextStateBc;
        case (StateBc)
		    WaitBc:     begin
							NBufClrToDo <= NBufClrToDo + BufClrAll0 + BufClrAll1 + BufClrAll2 + BufClrAll3;
			            end
            SendBc:		begin
                            BufClrAll <= 1'b1;
							$display("%g\t TkrDataMerge %d: sending buffer clear signal.  Number in queue=%d",$time,Address,NBufClrToDo);
							NBufClrToDo <= NBufClrToDo + BufClrAll0 + BufClrAll1 + BufClrAll2 + BufClrAll3 - 1;
						end
			PausBc:     begin
							NBufClrToDo <= NBufClrToDo + BufClrAll0 + BufClrAll1 + BufClrAll2 + BufClrAll3;
			                BufClrAll <= 1'b0;
						end
        endcase
    end
end

//State machine to find the next chip that has clusters.
parameter [2:0] WaitFd=3'b001;
parameter [2:0] LookFd=3'b010;
parameter [3:0] DoneFd=3'b100;

wire [3:0] NextChipP1;
assign NextChipP1 = FindChip + 1;
always @ (StateFd or Trig or NextChipP1 or NClus0 or NClus1 or NClus2 or NClus3 or NClus4 or NClus5 or NClus6 or NClus7 or NClus8 or NClus9 or NClusA or NClusB) begin
    case (StateFd)
        WaitFd:     begin
                        if (Trig) NextStateFd = LookFd;
                        else NextStateFd = WaitFd;
                    end
        LookFd:     begin
                        case (NextChipP1)
                            4'h0:    if (NClus0>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'h1:    if (NClus1>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'h2:    if (NClus2>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'h3:    if (NClus3>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'h4:    if (NClus4>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'h5:    if (NClus5>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'h6:    if (NClus6>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'h7:    if (NClus7>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'h8:    if (NClus8>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'h9:    if (NClus9>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'ha:    if (NClusA>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            4'hb:    if (NClusB>0) NextStateFd = DoneFd; else NextStateFd = LookFd;
                            default: NextStateFd = DoneFd;
                        endcase
                    end
        DoneFd:     begin
                        NextStateFd = WaitFd;
                    end
        default:    begin
                        NextStateFd = WaitFd; 
                    end
    endcase
end

reg [2:0] StateFd, NextStateFd;
reg [3:0] Chip;
reg [3:0] NextChip, FindChip;
reg Trig;        //Pulse to start search for next chip
always @ (posedge Clock) begin
    if (Reset) begin
        StateFd <= WaitFd;
    end else begin
        StateFd <= NextStateFd;
        // if (Address==0 && (Trig || StateFd != WaitFd)) begin
            // $display("%g\t TkrDataMerge %d: StateFd=%b Chip=%d FindChip=%d NextChipP1=%d Trig=%b NClus=%d %d %d %d %d %d %d %d %d %d %d %d",$time,Address,StateFd,Chip,FindChip,NextChipP1,Trig,NClus0,NClus1,NClus2,NClus3,NClus4,NClus5,NClus6,NClus7,NClus8,NClus9,NClusA,NClusB);
        // end
        case (StateFd)
            WaitFd:    begin
                        FindChip <= Chip;
                    end
            LookFd:    begin
                        FindChip <= FindChip + 1;
                    end
            DoneFd:    begin
                        NextChip <= FindChip;
                    end
        endcase
    end
end

//Calculate a flag to indicate when all chips are ready with at least one event to send.  
wire [11:0] Ready;
assign Ready[0] = (nFull0 !=0  || !Mask[0]);
assign Ready[1] = (nFull1 !=0  || !Mask[1]);
assign Ready[2] = (nFull2 !=0  || !Mask[2]);
assign Ready[3] = (nFull3 !=0  || !Mask[3]);
assign Ready[4] = (nFull4 !=0  || !Mask[4]);
assign Ready[5] = (nFull5 !=0  || !Mask[5]);
assign Ready[6] = (nFull6 !=0  || !Mask[6]);
assign Ready[7] = (nFull7 !=0  || !Mask[7]);
assign Ready[8] = (nFull8 !=0  || !Mask[8]);
assign Ready[9] = (nFull9 !=0  || !Mask[9]);
assign Ready[10] = (nFullA !=0 || !Mask[10]);
assign Ready[11] = (nFullB !=0 || !Mask[11]);
assign AllReady = (Ready == 12'b111111111111);

//Keep count of how many send event requests have come in
reg [3:0] SendEvtPending;
always @ (posedge Clock) begin
    if (Reset) begin
        SendEvtPending <= 0;
    end else begin
        if (SendEvt && !DoneCRC) begin
            SendEvtPending <= SendEvtPending + 1;
            $display("%g\t TkrDataMerge %d: incrementing SendEvtPending to %d.",$time,Address,SendEvtPending+1);
        end 
        if (DoneCRC && !SendEvt) begin
            SendEvtPending <= SendEvtPending - 1;
            $display("%g\t TkrDataMerge %d: decrementing SendEvtPending to %d.",$time,Address,SendEvtPending-1);
        end
    end
end

//State machine to merge the events
parameter [8:0] Idle = 9'b000000001;     //Wait for all chips to be ready
parameter [8:0] WtSd = 9'b000000010;     //Wait for a send event signal if it hasn't already arrived
parameter [8:0] DcHd = 9'b000000100;     //Decode the chip data headers and send the start bit
parameter [8:0] StBt = 9'b000001000;     //Output the rest of the 6-bit start string
parameter [8:0] Hdrr = 9'b000010000;     //Output the event header
parameter [8:0] ChHd = 9'b000100000;     //Output a chip header
parameter [8:0] Clus = 9'b001000000;     //Output clusters
parameter [8:0] CRCD = 9'b010000000;     //Wait for the CRC calculation to finish
parameter [8:0] Spac = 9'b100000000;     //Delay 1 clock cycle before allowing a new readout to start
reg [8:0] State, NextState;

//All of the timing in this state machine is set up to merge the data streams of the
//chips one after the other, with no breaks in between.  Chips with no data do not
//contribute to the stream (not even a header), to minimize data volume and readout time.
reg [3:0] Cnt;
reg [3:0] CntH;
reg [3:0] CntClus;
always @ (State or AllReady or SendEvtPending or Cnt or CntH or CntClus or DoneCRC or shiftOut or NextChip or NChipWithClus or NClusLat) begin
  case (State)
    Idle:     begin
                if (AllReady)
                    if (SendEvtPending > 0) NextState = DcHd;
                    else NextState = WtSd;
                else
                    NextState = Idle;
                MergeData = 1'b0;
            end
    WtSd:     begin
                if (SendEvtPending > 0) NextState = DcHd;
                else NextState = WtSd;
                MergeData = 1'b0;
            end
    DcHd:    begin
                NextState = StBt;
                MergeData = shiftOut[11];
            end
    StBt:    begin
                if (Cnt == 4) NextState = Hdrr;
                else NextState = StBt;
                MergeData = shiftOut[11];
            end
    Hdrr:    begin
                if (CntH == 11) begin
                    if (NChipWithClus>0) NextState = ChHd;
                    else NextState = CRCD;
                end else NextState = Hdrr;
                MergeData = shiftOut[11];
            end
    ChHd:    begin
                if (Cnt == 11) NextState = Clus;
                else NextState = ChHd;
                MergeData = shiftOut[11];
            end
    Clus:    begin
                if (CntH == 11) begin
                    if (CntClus == NClusLat) begin
                        if (NextChip > 11) NextState = CRCD;
                        else NextState = ChHd;
                    end else begin
                        NextState = Clus;
                    end
                end else begin
                    NextState = Clus;
                end
                MergeData = shiftOut[11];
            end
    CRCD:    begin
                if (Cnt == 0) MergeData = shiftOut[11];
                else MergeData = 1'b0;
                if (DoneCRC) NextState = Spac;
                else NextState = CRCD;                
            end
    Spac:    begin
                MergeData = 1'b0;
                NextState = Idle;
            end
    default:     begin 
                    NextState= Idle; 
                    MergeData = 1'b0;
                end
  endcase
end

reg [11:0] TagB1,TagB0;
reg [15:0] OvrFlw, ChipErr; //Overdimensioned such that can be addressed by 4 bits
wire [15:0] PrtyErr;        //Overdimensioned such that can be addressed by 4 bits
reg [3:0] NClus0, NClus1, NClus2, NClus3, NClus4, NClus5, NClus6, NClus7, NClus8, NClus9, NClusA, NClusB;
wire [3:0] NChipWithClus;

//This following addition does not have to complete in one clock period.  In fact, it has about 5 periods available to settle to the answer.
assign NChipWithClus = (NClus0>0) + (NClus1>0) + (NClus2>0) + (NClus3>0) + (NClus4>0) + (NClus5>0) + (NClus6>0) + (NClus7>0) + (NClus8>0) + (NClus9>0) + (NClusA>0) + (NClusB>0);

reg [11:0] shiftOut;

//Flag a tag error if the two tag bits don't agree between all 12 chips
assign TagErr = (TagB1 != 12'b000000000000 && TagB1 != 12'b111111111111) || (TagB0 != 12'b000000000000 && TagB0 != 12'b111111111111);

reg [3:0] NClusters, NClusLat;
always @ (Chip or NClus0 or NClus1 or NClus2 or NClus3 or NClus4 or NClus5 or NClus6 or NClus7 or NClus8 or NClus9 or NClusA or NClusB) begin
    case (Chip)
        4'h0:    NClusters = NClus0;
        4'h1:    NClusters = NClus1;
        4'h2:    NClusters = NClus2;
        4'h3:    NClusters = NClus3;
        4'h4:    NClusters = NClus4;
        4'h5:    NClusters = NClus5;
        4'h6:    NClusters = NClus6;
        4'h7:    NClusters = NClus7;
        4'h8:    NClusters = NClus8;
        4'h9:    NClusters = NClus9;
        4'ha:    NClusters = NClusA;
        4'hb:    NClusters = NClusB;
        default: NClusters = 0;
    endcase
end

reg [15:0] Nevent;
always @ (posedge Clock) begin
  if (Reset) begin
    State <= Idle;
    EvtNum <= 0;
    Send <= 1'b0;
    Strobe <= 0;
    ErrBuf[47:24] <= 0;
    ErrBuf[11:0] <= 0;
    Trig <= 1'b0;
    ChipErr <= 0;
    OvrFlw <= 0;
    Stop <= 1'b0;
    Nevent <= 0;
  end else begin
      ErrBuf[11:0] <= PrtyErr[11:0] | ErrBuf[11:0];  
    State <= NextState;
    // if (State != Idle || AllReady == 1'b1) begin
        // $display("%g\t TkrDataMerge %d: State=%b NChipWithClus=%d Send=%b Strobe=%b Trig=%b Stop=%b NClusters=%d NClusLat=%d shiftOut=%b",$time,Address,State,NChipWithClus,Send,Strobe,Trig,Stop,NClusters,NClusLat,shiftOut);
        // $display("%g\t   >TkrDataMerge %d: CntClus=%d TagB1=%b TagB0=%b Chip=%d NextChip=%d ChipErr=%b OvrFlw=%b MergeData=%b Cnt=%d CntH=%d Ready=%b",$time,Address,CntClus,TagB1,TagB0,Chip,NextChip,ChipErr,OvrFlw,MergeData,Cnt,CntH,Ready);
        // $display("%g\t   >TkrDataMerge %d: Number of clusters=%d %d %d %d %d %d %d %d %d %d %d %d MergeDataOut=%b DoneCRC=%b",$time,Address,NClus0,NClus1,NClus2,NClus3,NClus4,NClus5,NClus6,NClus7,NClus8,NClus9,NClusA,NClusB,MergeDataOut,DoneCRC);
        // $display("%g\t   >TkrDataMerge %d: DataOut=%b %b %b %b %b %b %b %b %b %b %b %b",$time,Address,DataOut0,DataOut1,DataOut2,DataOut3,DataOut4,DataOut5,DataOut6,DataOut7,DataOut8,DataOut9,DataOutA,DataOutB);
    // end
    case (State)
      Idle: begin
                if (AllReady) Send <= 1'b1;  //Get the first word (data header) from all chips
                Cnt <= 0;
                CntH <= 0;
                Strobe <= 0;
                shiftOut <= 0;
            end
      WtSd:    begin
                Send <= 1'b0;
            end
      DcHd: begin
                $display("%g\t TkrDataMerge %d: Begin sending serial stream, EvtNum= %d",$time,Address,EvtNum);
                Send <= 1'b0;
                Chip <= 4'b1111;
                shiftOut[11:6] <= {2'b10,Address};
            end
      StBt:    begin
                if (Cnt == 0) begin    //Store all of the header information from the 12 chips
                    $display("%g\t TkrDataMerge %d: event with tag=%d is going out.",$time,Address,DataOut0[9:8]);
                    TagB1[0] <= DataOut0[9];
                    TagB1[1] <= DataOut1[9];
                    TagB1[2] <= DataOut2[9];
                    TagB1[3] <= DataOut3[9];
                    TagB1[4] <= DataOut4[9];
                    TagB1[5] <= DataOut5[9];
                    TagB1[6] <= DataOut6[9];
                    TagB1[7] <= DataOut7[9];
                    TagB1[8] <= DataOut8[9];
                    TagB1[9] <= DataOut9[9];
                    TagB1[10] <= DataOutA[9];
                    TagB1[11] <= DataOutB[9];
                    TagB0[0] <= DataOut0[8];
                    TagB0[1] <= DataOut1[8];
                    TagB0[2] <= DataOut2[8];
                    TagB0[3] <= DataOut3[8];
                    TagB0[4] <= DataOut4[8];
                    TagB0[5] <= DataOut5[8];
                    TagB0[6] <= DataOut6[8];
                    TagB0[7] <= DataOut7[8];
                    TagB0[8] <= DataOut8[8];
                    TagB0[9] <= DataOut9[8];
                    TagB0[10] <= DataOutA[8];
                    TagB0[11] <= DataOutB[8];
                    ChipErr[0] <= DataOut0[7];
                    ChipErr[1] <= DataOut1[7];
                    ChipErr[2] <= DataOut2[7];
                    ChipErr[3] <= DataOut3[7];
                    ChipErr[4] <= DataOut4[7];
                    ChipErr[5] <= DataOut5[7];
                    ChipErr[6] <= DataOut6[7];
                    ChipErr[7] <= DataOut7[7];
                    ChipErr[8] <= DataOut8[7];
                    ChipErr[9] <= DataOut9[7];
                    ChipErr[10] <= DataOutA[7];
                    ChipErr[11] <= DataOutB[7];
                    OvrFlw[0] <= DataOut0[5];
                    OvrFlw[1] <= DataOut1[5];
                    OvrFlw[2] <= DataOut2[5];
                    OvrFlw[3] <= DataOut3[5];
                    OvrFlw[4] <= DataOut4[5];
                    OvrFlw[5] <= DataOut5[5];
                    OvrFlw[6] <= DataOut6[5];
                    OvrFlw[7] <= DataOut7[5];
                    OvrFlw[8] <= DataOut8[5];
                    OvrFlw[9] <= DataOut9[5];
                    OvrFlw[10] <= DataOutA[5];
                    OvrFlw[11] <= DataOutB[5];    
                    NClus0 <= DataOut0[3:0];
                    NClus1 <= DataOut1[3:0];
                    NClus2 <= DataOut2[3:0];
                    NClus3 <= DataOut3[3:0];
                    NClus4 <= DataOut4[3:0];
                    NClus5 <= DataOut5[3:0];
                    NClus6 <= DataOut6[3:0];
                    NClus7 <= DataOut7[3:0];
                    NClus8 <= DataOut8[3:0];
                    NClus9 <= DataOut9[3:0];
                    NClusA <= DataOutA[3:0];
                    NClusB <= DataOutB[3:0];
                end
                Cnt <= Cnt + 1;
                Trig <= (Cnt == 1);  //Start search for the first chip with clusters
                shiftOut <= {shiftOut[10:0],1'b0};  //Shift out the start string
            end
      Hdrr:    begin
                Cnt <= 0;
                CntH <= CntH + 1;
                if (CntH == 0) begin
                    $display("%g\t TkrDataMerge %d: sending header %d %b%b %b %d.",$time,Address,EvtNum,TagB1[0],TagB0[0],TagErr,NChipWithClus);
                    shiftOut <= {EvtNum,TagB1[0],TagB0[0],TagErr,NChipWithClus}; 
                    if (TagErr) ErrBuf[47:24] <= {TagB1,TagB0};
                    if (TagB1[0] & TagB0[0]) EvtNum <= EvtNum + 1;   //Increment the event number in synch with the tag
                end else begin
                    shiftOut <= {shiftOut[10:0],1'b0};    //Shift out the header
                end
                if (CntH == 11) Chip <= NextChip;
                if (NextState == CRCD) Stop <= 1'b1;
            end
      ChHd:    begin
                if (Cnt == 0) begin
                    shiftOut <= {OvrFlw[Chip],1'b0,NClusters,ChipErr[Chip],PrtyErr[Chip],Chip};
                    Trig <= 1'b1;    //Start looking for the next chip with clusters
                    Strobe[Chip] <= 1'b1;    //Get the first cluster data word for this chip
                    NClusLat <= NClusters;
                end else begin
                    Strobe <= 0;
                    Trig <= 1'b0;
                    shiftOut <= {shiftOut[10:0],1'b0};    //Shift out the ASIC header        
                end
                Cnt <= Cnt + 1;
                CntH <= 0;
                CntClus <= 0;
            end
      Clus:    begin
                Cnt <= 0;
                if (CntH == 11) begin
                    CntH <= 0;
                    if (CntClus == NClusLat) begin
                        Chip <= NextChip;
                    end
                end else begin
                    CntH <= CntH + 1;
                end
                if (CntH == 0) begin
                    CntClus <= CntClus + 1;
                    case (Chip)
                        4'h0:    shiftOut <= DataOut0;
                        4'h1:    shiftOut <= DataOut1;
                        4'h2:    shiftOut <= DataOut2;
                        4'h3:    shiftOut <= DataOut3;
                        4'h4:    shiftOut <= DataOut4;
                        4'h5:    shiftOut <= DataOut5;
                        4'h6:    shiftOut <= DataOut6;
                        4'h7:    shiftOut <= DataOut7;
                        4'h8:    shiftOut <= DataOut8;
                        4'h9:    shiftOut <= DataOut9;
                        4'ha:    shiftOut <= DataOutA;
                        4'hb:    shiftOut <= DataOutB;
                    endcase
                    if (CntClus < NClusLat-1) Strobe[Chip] <= 1'b1;    //Get the next cluster data word
                end else begin
                    Strobe[Chip] <= 1'b0;
                    shiftOut <= {shiftOut[10:0],1'b0};    //Shift out a cluster word
                end
                if (NextState == CRCD) Stop <= 1'b1;
            end
      CRCD:    begin
                Cnt <= Cnt + 1;
                Stop <= 1'b0;
                if (DoneCRC) Strobe <= 12'b111111111111;    //Get the data sending machines ready to go for the next event
            end
      Spac:    begin
                $display("%g\t TkrDataMerge %d: completed sending of event %d.",$time,Address,Nevent);
                Nevent <= Nevent + 1;
            end
    endcase
  end
end

endmodule

module BufClrTkr(ErrBuf,BufClrAll,BufClr,Mask,Clock,Reset);
input Clock;
input Reset;
input [11:0] BufClr;       //Single-clock pulse each time a chip has finished sending an event
input [11:0] Mask;         //To mask out individual chips
output[11:0] ErrBuf;       //Chip error status
output BufClrAll;          //Single-clock pulse when all 12 chips have finished

reg BufClrAll;
reg [11:0] ErrBuf;

reg [11:0] ChipsDone;
reg [11:0] CntTime;
always @ (posedge Clock)
begin
  if (Reset) begin
    ChipsDone <= 0;
    CntTime <= 0;
    ErrBuf <= 0;
  end else begin
    if ((ChipsDone | ~Mask) == 12'b111111111111) begin
      BufClrAll <= 1'b1;
//      $display("%g\t TkrDatMerge %d: buffer clear signal going out.  ChipsDone=%b; CntTime=%d, BufClr=%b",$time,Address,ChipsDone,CntTime,BufClr);
      ChipsDone <= BufClr;
      CntTime <= 0;
    end else begin
      ChipsDone <= ChipsDone | BufClr;
//      if (BufClr != 0) $display("%g\t TkrDatMerge %d: BufClr input=%b; ChipsDone=%b",$time,Address,BufClr,ChipsDone);
      BufClrAll <= 1'b0;
      if (CntTime == 12'b111111111111) begin  //All chips should have reported by now!
        ErrBuf <= ~(ChipsDone | ~Mask);   //Save status, showing which chips are not reporting
      end else begin
        if (ChipsDone != 0) CntTime <= CntTime + 1;  //Start counting as soon as at least one chip reports
      end
    end
  end
end

endmodule
