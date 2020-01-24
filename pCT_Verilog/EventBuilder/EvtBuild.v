// Build events from 14 input serial streams.
// Data stream into EvtBuf instances in parallel.
// When all 16 are ready (BufRdy), this program cycles through all 16 sequentially, signaling them one
// at a time to send data out in 12-bit words. 
// R. Johnson
// Modified 6/10/2013 to increment the event counter only after all bits have been received
// Modified 6/11/2013 to add an extended header to aid in event recognition
// Modified 6/28/2013 to work with any subset of all 14 boards
// Modified 7/9/2013 to include the layer number in the header of missing layers
// Modified 8/11/2013 to fill the error flags in the header
// Modified 8/27/2013 to remove unused error signal from energy detector buffers
// Modified 9/26/2013 to add reporting of tag errors back to the event builder
// Modified 10/3/2013 to add the EthHold input
// Modified 10/31/2013, changing the tracker buffer limit from 7 to 8, for the HoldTrig flag.
// Modified 1/25/2014 to make the event end on a byte boundary
// Modified 1/29/2014 to use a second parallel data stream for reduced events with pedestals 
//                           (needs more mods then to work still for non-reduced events or events without pedestals)
// Modified 4/2/2014 to rework the trigger management
// Modified 4/25/2014 to precalculate error flags to avoid timing issues

module EvtBuild(Trigger, Bufree, BufRdy,EvStart, TagErr, CRCerr, NFull0, HoldTrig, NflC, NEvent, DAQerr, AddErr, HoldSnd, DataOut, Strobe, Clock, Reset, DataIn, MaskIn, EthHold, DeltaT, TimeTag, TagOut, BufMngrType, TrgBitsOut, TrgWdOut);
input BufMngrType;		//Old or new buffer management scheme
input Trigger;			//Trigger accept pulse
input TagOut;			//True to output the time tag in each event header
input [11:0] DeltaT;	//Time since the previous trigger generated
input [35:0] TimeTag;	//Time since the start of run
input EthHold;			//Held high when the Ethernet card cannot accept more events
input [15:0] MaskIn;	//Select which input data streams to use
input [15:0] DataIn;	//Input serial data streams
input Clock, Reset;
output [11:0] DataOut;  //Output 12-bit parallel data stream
output Strobe;			//Signal set when valid data are in DataOut
output HoldSnd;			//Signal to stop pulling events from front-end FPGAs, as local buffers are getting full
output DAQerr;			//1-clock signal if a DAQ error was encountered
output [15:0] AddErr;	//Bit is set if a datastream address doesn't match what is expected
output [31:0] NEvent;	//Event count for end-of-run summary
output [3:0] NflC;		//Number of buffers full in the first front-end energy detector board, for monitoring
output HoldTrig;		//Signal to hold the trigger to prevent overflow of front-end-board FPGA buffers
output [4:0] NFull0;	//Number of local buffers full, out of 16, for monitoring
output TagErr;			//Signal that a Tag error has occured.
output CRCerr;			//Signal that a CRC error has occured.  
output EvStart;			//One clock signal that a new event is being built
output [15:0] BufRdy;	//Ready status of the output buffers
output [15:0] Bufree;   //Status of front-end board buffers
input TrgBitsOut;       //Set to replace 5 bits in the event number by energy detector trigger bits
input [5:0] TrgWdOut;   //6 trigger bits from the energy detector

wire CRCerr;
reg [11:0] DataOut;
wire [4:0] NFull0, NFull1, NFull2, NFull3, NFull4, NFull5, NFull6, NFull7, NFull8, NFull9, NFullA, NFullB, NFullC, NFullD, NFullE, NFullF;
wire [7:0] ErrBuf0, ErrBuf1, ErrBuf2, ErrBuf3, ErrBuf4, ErrBuf5, ErrBuf6, ErrBuf7, ErrBuf8, ErrBuf9, ErrBufA, ErrBufB, ErrBufC, ErrBufD, ErrBufE, ErrBufF;
reg Strobe;
reg TagErr;

wire [11:0] DataOut0,DataOut1,DataOut2,DataOut3,DataOut4,DataOut5,DataOut6,DataOut7,DataOut8,DataOut9,DataOutA,DataOutB,DataOutC,DataOutD,DataOutE,DataOutF;
wire [15:0] BufRdy, SendHold, AddErr, CRCbad;
wire [11:0] ErrSgnl;
reg [16:0] Send;
reg [31:0] NEvent;
reg [3:0] Error;
reg SendErrBuf;

assign HoldTrig = ((Bufree | ~Mask[15:0]) != 16'b1111111111111111);   	

wire [16:0] Mask;
assign Mask = {1'b0,MaskIn[15] & TwoLinks, MaskIn[14], MaskIn[13] & TwoLinks, MaskIn[12:0]};   //Add an extra bit to the mask just to simplify the pointer logic below
wire [13:0] MaskErr;
assign MaskErr = {MaskIn[14],MaskIn[12:0]};

// State machine to pull data from the event buffers and send an event to the output stream
parameter [11:0] Wait = 12'b000000000001;	//Wait for all buffers to be ready
parameter [11:0] EHd1 = 12'b000000000010;
parameter [11:0] EHd2 = 12'b000000000100;
parameter [11:0] Tag1 = 12'b000000001000; 	//Output the first 12 bits of the event tag
parameter [11:0] Tag2 = 12'b000000010000; 	//Output the second 12 bits
parameter [11:0] Tag3 = 12'b000000100000; 	//Output the last 12 bits
parameter [11:0] DelT = 12'b000001000000; 	//Output the trigger delta-T
parameter [11:0] Hdr1 = 12'b000010000000;		
parameter [11:0] Hdr2 = 12'b000100000000;		
parameter [11:0] Hdr3 = 12'b001000000000; 		
parameter [11:0] Gett = 12'b010000000000;	//Output the events
parameter [11:0] Done = 12'b100000000000;

reg [11:0] State, NextState;
reg [11:0] Header;
reg [4:0] Isend, Inext;
reg [7:0] Icnt, NWords;
reg [1:0] TagOld;
wire [1:0] TagOldP1;
assign TagOldP1 = TagOld + 1;
reg oddEven;					//Used to keep track of whether an even number of 12-bit words went out

//always @ (BufRdy or State) begin
//	if (BufRdy != 0) $display("%g\t @EvtBuild: State=%b, BufRdy changes to %b",$time,State,BufRdy);
//end

always @ (State or oddEven or TwoLinks or EthHold or BufRdy or Header or DataOut0 or DataOut1 or DataOut2 or DataOut3 or DataOut4 or DataOut5 or DataOut6 or DataOut7 or DataOut8 or DataOut9 or DataOutA or DataOutB or DataOutC or DataOutD or DataOutE or DataOutF or NEvent or Isend or NWords or Icnt or Mask or TagOut or DeltaT or TimeTag) begin
	case (State) 
		Wait:		begin
						if ((BufRdy | ~Mask[15:0]) == 16'b1111111111111111 && !EthHold) NextState = EHd1;
						else NextState = Wait;
						DataOut = 0;
						Strobe = 1'b0;
					end
		EHd1:		begin
						DataOut = 0;
						Strobe = 1'b0;
						NextState = EHd2;
					end
		EHd2:		begin
						DataOut = Header;
						Strobe = 1'b1;
						if (TagOut) NextState = Tag1;
						else NextState = DelT;
					end
		Tag1:		begin
						Strobe = 1'b1;
						DataOut = Header;
						NextState = Tag2;
					end
		Tag2:		begin
						Strobe = 1'b1;
						DataOut = Header;
						NextState = Tag3;
					end
		Tag3:		begin
						Strobe = 1'b1;
						DataOut = Header;
						NextState = DelT;
					end
		DelT:		begin
						Strobe = 1'b1;
						DataOut = Header;
						NextState = Hdr1;
					end
		Hdr1:		begin
						DataOut = Header;
						Strobe = 1'b1;
						NextState = Hdr2;
					end
		Hdr2:		begin
						Strobe = 1'b1;
						DataOut = Header;
						NextState = Hdr3;
					end
		Hdr3:		begin
						Strobe = 1'b1;
						DataOut = Header;
						NextState = Gett;
					end
		Gett:		begin
						if (NWords >= 3) Strobe = 1'b1;
						else begin
							if (Mask[Isend]) Strobe = (Icnt <= NWords);
							else Strobe = (Icnt == 2);
						end
						if (Mask[Isend]) begin
							case (Isend)
								4'h0: DataOut = DataOut0;
								4'h1: DataOut = DataOut1;
								4'h2: DataOut = DataOut2;
								4'h3: DataOut = DataOut3;
								4'h4: DataOut = DataOut4;
								4'h5: DataOut = DataOut5;
								4'h6: DataOut = DataOut6;
								4'h7: DataOut = DataOut7;
								4'h8: DataOut = DataOut8;
								4'h9: DataOut = DataOut9;
								4'hA: DataOut = DataOutA;
								4'hB: DataOut = DataOutB;
								4'hC: DataOut = DataOutC;
								4'hD: DataOut = DataOutD;
								4'hE: DataOut = DataOutE;
								4'hF: DataOut = DataOutF;
								default: DataOut = 0;
							endcase
						end else DataOut = {Isend,NEvent[2:0],5'b00000};   //For layers not attached, just send out a header word
						if (Icnt >= NWords) begin
							if (Isend == 15 || (!TwoLinks && Isend == 14)) NextState = Done;
							else NextState = Gett;
						end	else NextState = Gett;
					end
		Done:		begin
						if (oddEven) begin
							Strobe = 1'b1;		//Send out an extra empty 12-bit word so that the next event starts on a Byte boundary.
						end else begin
							Strobe = 1'b0;
						end
						DataOut = 0;
						NextState = Wait;
					end
		default:	begin NextState = Wait; DataOut = 0; Strobe = 1'b0; end
	endcase
end

//always @ (BufRdy or State or Mask) begin
//	$display("%g\t    @EvtBuild:  State=%b, BufRdy=%b, Mask=%b",$time,State,BufRdy,Mask[13:0]);
//end
reg [15:0] Error1,Error3;
reg [13:0] Error2;
reg [11:0] Error0;
assign EvStart = (State == EHd1);
always @ (posedge Clock) begin
	if (Reset) begin
		NEvent <= 0;
		Send <= 1'b0;
		State <= Wait;
		Error <= 0;
		SendErrBuf <= 1'b0;
		TagOld <= 2'b11;
		TagErr <= 1'b0;
	end else begin
		State <= NextState;
		if (State != Wait) begin
//			$display("%g\t @EvtBuild: State=%b, Header=%b, NWords=%d, Icnt=%d, Isend=%d, DataOut=%b, Strobe=%b.",$time,State,Header,NWords,Icnt,Isend,DataOut,Strobe);
			if (Strobe) oddEven <= !oddEven;    //Keep track of whether an even number of 12-bit words goes out in this event
		end
		case (State)
			Wait:	begin
						if ((BufRdy | ~Mask[15:0]) == 16'b1111111111111111 && !EthHold) begin
							$display("%g\t @EvtBuild Wait:  all buffers ready, NEvent=%d, BufRdy=%b, Mask=%b",$time,NEvent,BufRdy,Mask);	
							SendErrBuf <= 1'b1;
						end
						if (EthHold) $display("%g\t @EvtBuild Wait: Ethernet hold at event %d.  Buffer ready status=%b, Mask=%b",$time,NEvent,BufRdy,Mask);
						oddEven <= 1'b0;		//Initialize this at the event beginning
					end
			EHd1:	begin
						Header <= 12'b111100000100;
						SendErrBuf <= 1'b0;
					end
			EHd2:	begin
						Header <= 12'b001101010100;
						Error3[0] <= ErrBuf0[1];      //Precalculate some error flags
						Error3[1] <= ErrBuf1[1];
						Error3[2] <= ErrBuf2[1];
						Error3[3] <= ErrBuf3[1];
						Error3[4] <= ErrBuf4[1];
						Error3[5] <= ErrBuf5[1];
						Error3[6] <= ErrBuf6[1];
						Error3[7] <= ErrBuf7[1];
						Error3[8] <= ErrBuf8[1];
						Error3[9] <= ErrBuf9[1];
						Error3[10] <= ErrBufA[1];
						Error3[11] <= ErrBufB[1];
						Error3[12] <= ErrBufC[1];
						Error3[13] <= ErrBufD[1];
						Error3[14] <= ErrBufE[1];
						Error3[15] <= ErrBufF[1];
						Error2[0] <= (ErrBuf0[4] != TagOldP1[1]) || (ErrBuf0[3] != TagOldP1[0]);
						Error2[1] <= ErrBuf1[6:3] != ErrBuf0[6:3];
						Error2[2] <= ErrBuf2[6:3] != ErrBuf0[6:3];
						Error2[3] <= ErrBuf3[6:3] != ErrBuf0[6:3];
						Error2[4] <= ErrBuf4[6:3] != ErrBuf0[6:3];
						Error2[5] <= ErrBuf5[6:3] != ErrBuf0[6:3];
						Error2[6] <= ErrBuf6[6:3] != ErrBuf0[6:3];
						Error2[7] <= ErrBuf7[6:3] != ErrBuf0[6:3];
						Error2[8] <= ErrBuf8[6:3] != ErrBuf0[6:3];
						Error2[9] <= ErrBuf9[6:3] != ErrBuf0[6:3];
						Error2[10] <= ErrBufA[6:3] != ErrBuf0[6:3];
						Error2[11] <= ErrBufB[6:3] != ErrBuf0[6:3];
						Error2[12] <= ErrBufC[6:3] != ErrBuf0[6:3];
						Error2[13] <= ErrBufE[6:3] != ErrBuf0[6:3];
						Error1[0] <= ErrBuf0[0];
						Error1[1] <= ErrBuf1[0];
						Error1[2] <= ErrBuf2[0];
						Error1[3] <= ErrBuf3[0];
						Error1[4] <= ErrBuf4[0];
						Error1[5] <= ErrBuf5[0];
						Error1[6] <= ErrBuf6[0];
						Error1[7] <= ErrBuf7[0];
						Error1[8] <= ErrBuf8[0];
						Error1[9] <= ErrBuf9[0];
						Error1[10] <= ErrBufA[0];
						Error1[11] <= ErrBufB[0];
						Error1[12] <= ErrBufC[0];
						Error1[13] <= ErrBufD[0];
						Error1[14] <= ErrBufE[0];
						Error1[15] <= ErrBufF[0];
						Error0[0] <= ErrBuf0[2];
						Error0[1] <= ErrBuf1[2];
						Error0[2] <= ErrBuf2[2];
						Error0[3] <= ErrBuf3[2];
						Error0[4] <= ErrBuf4[2];
						Error0[5] <= ErrBuf5[2];
						Error0[6] <= ErrBuf6[2];
						Error0[7] <= ErrBuf7[2];
						Error0[8] <= ErrBuf8[2];
						Error0[9] <= ErrBuf9[2];
						Error0[10] <= ErrBufA[2];
						Error0[11] <= ErrBufB[2];
					end
			Tag1:	begin
						Header <= TimeTag[35:24];
					end
			Tag2:	begin
						Header <= TimeTag[23:12];
					end
			Tag3:	begin
						Header <= TimeTag[11:0];
					end
			DelT:	begin
						Header <= DeltaT;
						TagOld <= ErrBuf0[4:3];
						Error[3] <= (Mask[14:0] & Error3) != 16'b0000000000000000;   
						Error[2] <= (MaskErr & Error2) != 14'b00000000000000;   //Tag mismatch 
						Error[1] <= (Mask[14:0] & Error1) != 16'b0000000000000000; 	//Bad CRC
						Error[0] <= (Mask[11:0] & Error0) != 12'b000000000000;				
					end
			Hdr1:	begin				
						TagErr <= Error[2];
						if (TrgBitsOut) Header <= {2'b10,Error,TrgWdOut};
						else Header <= {2'b10,Error,NEvent[17:12]};
						if (Mask[0]) Send[0] <= 1'b1;
						$display("%g\t @EvtBuild for event %d, Error flags=%b",$time,NEvent,Error);
					end
			Hdr2:	begin		
						TagErr <= 1'b0;
						Send <= 0;
						Header <= NEvent[11:0];
						$display("%g\t @EvtBuild Hdr2: DataOut=%b, NEvent=%d",$time,DataOut,NEvent);
					end
			Hdr3:	begin
						NWords <= DataOut0;
						Icnt <= 1;
						Isend <= 0;
						Inext <= 1;
						$display("%g\t @EvtBuild Hdr3:  DataOut=%b, DataOut0=%b",$time,DataOut,DataOut0);
					end
			Gett:	begin   
//						$display("%g\t @EvtBuild Gett: DataOut=%b %b %b %b, NWords=%d, Isend=%d, Icnt=%d, Send=%b, Strobe=%b",$time,DataOut,DataOut0,DataOut1,DataOutC,NWords,Isend,Icnt,Send,Strobe);
						if (NWords < 3) begin    //In this case we have to skip 1 or 2 clock cycles with no data output
//							$display("%g\t EvtBuild Gett:  example with Nwords<3",$time);
							if (Icnt == 1 && Mask[Inext]) begin
								Send[Inext] <= 1'b1;
//								$display("%g\t @EvtBuild state Gett: Event=%d, Icnt=%d, Isend=%d, Inext=%d, Send=%b, Mask=%b, NWords=%d",$time,NEvent,Icnt,Isend,Inext, Send, Mask, NWords); 
							end else Send <= 0;
							if (Icnt == 3) begin
								Icnt <= 1;
								Isend <= Inext;
								if (Inext == 12 && !TwoLinks) Inext <= 14;
								else Inext <= Inext + 1;
								if (Mask[Inext]) begin
									case (Inext)
										4'h1: NWords <= DataOut1;
										4'h2: NWords <= DataOut2;
										4'h3: NWords <= DataOut3;
										4'h4: NWords <= DataOut4;
										4'h5: NWords <= DataOut5;
										4'h6: NWords <= DataOut6;
										4'h7: NWords <= DataOut7;
										4'h8: NWords <= DataOut8;
										4'h9: NWords <= DataOut9;
										4'hA: NWords <= DataOutA;
										4'hB: NWords <= DataOutB;
										4'hC: NWords <= DataOutC;
										4'hD: NWords <= DataOutD;
										4'hE: NWords <= DataOutE;
										4'hF: NWords <= DataOutF;
									endcase
								end else NWords <= 2;
							end else Icnt <= Icnt + 1;
						end else begin
							if (Icnt == NWords - 2 && Mask[Inext]) begin
								Send[Inext] <= 1'b1;
//								$display("%g\t @EvtBuild state Gett: Event=%d, Icnt=%d, Isend=%d, Inext=%d, Send=%b, Mask=%b, NWords=%d",$time,NEvent,Icnt,Isend,Inext, Send, Mask, NWords); 
							end else Send <= 0;
							if (Icnt == NWords) begin
								Icnt <= 1;
								Isend <= Inext;
								if (Inext == 12 && !TwoLinks) Inext <= 14;
								else Inext <= Inext + 1;
								if (Mask[Inext]) begin
									case (Inext)
										4'h1: NWords <= DataOut1;
										4'h2: NWords <= DataOut2;
										4'h3: NWords <= DataOut3;
										4'h4: NWords <= DataOut4;
										4'h5: NWords <= DataOut5;
										4'h6: NWords <= DataOut6;
										4'h7: NWords <= DataOut7;
										4'h8: NWords <= DataOut8;
										4'h9: NWords <= DataOut9;
										4'hA: NWords <= DataOutA;
										4'hB: NWords <= DataOutB;
										4'hC: NWords <= DataOutC;
										4'hD: NWords <= DataOutD;
										4'hE: NWords <= DataOutE;
										4'hF: NWords <= DataOutF;
									endcase
								end else NWords <= 2;
							end else Icnt <= Icnt + 1;
						end
					end
			Done:	begin	
						NEvent <= NEvent + 1;
					end
		endcase
	end
end

assign HoldSnd = ((SendHold & Mask[15:0]) != 0);
assign DAQerr = ((ErrSgnl & Mask[11:0]) != 0);
assign CRCerr = ((CRCbad & Mask[15:0]) != 0);
always @ (posedge HoldSnd) $display("%g\t @EvtBuild:  Hold signal occurance.",$time);

assign TwoLinks = TwoLinksC || TwoLinksE;

wire [3:0] Nfl0,Nfl1,Nfl2,Nfl3,Nfl4,Nfl5,Nfl6,Nfl7,Nfl8,Nfl9,NflA,NflB,NflC,NflD,NflE,NflF;
wire [15:0] Bufree;		//Status of front-end buffer occupancies
always @ (Bufree) begin
	$display("%g\t @EvtBuild: front-end-board occupancy flags=%b",$time,Bufree);
end
// Modules to buffer the data coming in from each of the 14 FPGAs.  The energy detector FPGAs each have two data streams.
EvtBuf12 EvtBuf0(ErrBuf0,CRCbad[0],Bufree[0],Nfl0,NFull0,ErrSgnl[0],BufRdy[0],DataOut0,SendHold[0],AddErr[0], Clock, Reset, DataIn[0], 4'h0, Send[0], SendErrBuf, Trigger & Mask[0], BufMngrType);
EvtBuf12 EvtBuf1(ErrBuf1,CRCbad[1],Bufree[1],Nfl1,NFull1,ErrSgnl[1],BufRdy[1],DataOut1,SendHold[1],AddErr[1], Clock, Reset, DataIn[1], 4'h1, Send[1], SendErrBuf, Trigger & Mask[1], BufMngrType);
EvtBuf12 EvtBuf2(ErrBuf2,CRCbad[2],Bufree[2],Nfl2,NFull2,ErrSgnl[2],BufRdy[2],DataOut2,SendHold[2],AddErr[2], Clock, Reset, DataIn[2], 4'h2, Send[2], SendErrBuf, Trigger & Mask[2], BufMngrType);
EvtBuf12 EvtBuf3(ErrBuf3,CRCbad[3],Bufree[3],Nfl3,NFull3,ErrSgnl[3],BufRdy[3],DataOut3,SendHold[3],AddErr[3], Clock, Reset, DataIn[3], 4'h3, Send[3], SendErrBuf, Trigger & Mask[3], BufMngrType);
EvtBuf12 EvtBuf4(ErrBuf4,CRCbad[4],Bufree[4],Nfl4,NFull4,ErrSgnl[4],BufRdy[4],DataOut4,SendHold[4],AddErr[4], Clock, Reset, DataIn[4], 4'h4, Send[4], SendErrBuf, Trigger & Mask[4], BufMngrType);
EvtBuf12 EvtBuf5(ErrBuf5,CRCbad[5],Bufree[5],Nfl5,NFull5,ErrSgnl[5],BufRdy[5],DataOut5,SendHold[5],AddErr[5], Clock, Reset, DataIn[5], 4'h5, Send[5], SendErrBuf, Trigger & Mask[5], BufMngrType);
EvtBuf12 EvtBuf6(ErrBuf6,CRCbad[6],Bufree[6],Nfl6,NFull6,ErrSgnl[6],BufRdy[6],DataOut6,SendHold[6],AddErr[6], Clock, Reset, DataIn[6], 4'h6, Send[6], SendErrBuf, Trigger & Mask[6], BufMngrType);
EvtBuf12 EvtBuf7(ErrBuf7,CRCbad[7],Bufree[7],Nfl7,NFull7,ErrSgnl[7],BufRdy[7],DataOut7,SendHold[7],AddErr[7], Clock, Reset, DataIn[7], 4'h7, Send[7], SendErrBuf, Trigger & Mask[7], BufMngrType);
EvtBuf12 EvtBuf8(ErrBuf8,CRCbad[8],Bufree[8],Nfl8,NFull8,ErrSgnl[8],BufRdy[8],DataOut8,SendHold[8],AddErr[8], Clock, Reset, DataIn[8], 4'h8, Send[8], SendErrBuf, Trigger & Mask[8], BufMngrType);
EvtBuf12 EvtBuf9(ErrBuf9,CRCbad[9],Bufree[9],Nfl9,NFull9,ErrSgnl[9],BufRdy[9],DataOut9,SendHold[9],AddErr[9], Clock, Reset, DataIn[9], 4'h9, Send[9], SendErrBuf, Trigger & Mask[9], BufMngrType);
EvtBuf12 EvtBufA(ErrBufA,CRCbad[10],Bufree[10],NflA,NFullA,ErrSgnl[10],BufRdy[10],DataOutA,SendHold[10],AddErr[10], Clock, Reset, DataIn[10], 4'hA, Send[10], SendErrBuf, Trigger & Mask[10], BufMngrType);
EvtBuf12 EvtBufB(ErrBufB,CRCbad[11],Bufree[11],NflB,NFullB,ErrSgnl[11],BufRdy[11],DataOutB,SendHold[11],AddErr[11], Clock, Reset, DataIn[11], 4'hB, Send[11], SendErrBuf, Trigger & Mask[11], BufMngrType);
EnrgBuf2 EvtBufC(TwoLinksC,ErrBufC,CRCbad[12],Bufree[12],NflC,NFullC,BufRdy[12],DataOutC,SendHold[12],AddErr[12], Clock, Reset, DataIn[12], 4'hC, Send[12], SendErrBuf, Trigger & Mask[12], BufMngrType);
EnrgBufData2 EvtBufD(ErrBufD, CRCbad[13],Bufree[13],NflD, NFullD, BufRdy[13], DataOutD, SendHold[13], AddErr[13], Clock, Reset, DataIn[13], 4'hC, Send[13], SendErrBuf, Trigger & Mask[13], BufMngrType);
EnrgBuf2 EvtBufE(TwoLinksE,ErrBufE,CRCbad[14],Bufree[14],NflE,NFullE,BufRdy[14],DataOutE,SendHold[14],AddErr[14], Clock, Reset, DataIn[14], 4'hD, Send[14], SendErrBuf, Trigger & Mask[14], BufMngrType);
EnrgBufData2 EvtBufF(ErrBufF, CRCbad[15],Bufree[15],NflF, NFullF, BufRdy[15], DataOutF, SendHold[15], AddErr[15], Clock, Reset, DataIn[15], 4'hD, Send[15], SendErrBuf, Trigger & Mask[15], BufMngrType);
endmodule
