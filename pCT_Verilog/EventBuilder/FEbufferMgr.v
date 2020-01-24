// Keep track of the front-end buffer occupancy of the pCT tracker ASICs, and let
// the data acquisition know when a free buffer is available in all chips.
// R. Johnson,  September 8, 2013
module FEbufferMgr(MaskIn,NoverFlow,NTagErr, NTimeOut, NprtyErr, BufClr, UnDoBufIn, Clock, Reset,
																Nclr0, Nclr1, Nclr2, Nclr3, Nclr4, Nclr5, Nclr6, Nclr7);
input Clock;
input Reset;
input [7:0] MaskIn;		// Which tracker layers are in the readout
input [7:0] UnDoBufIn;	// Serial stream from each of the 8 tracker boards
output [3:0] BufClr;	// Set true for one clock cycle when a buffer has cleared in all ASICs
output [15:0] NTagErr;	// Number of trigger tag errors
output [15:0] NTimeOut;	// Number of time-outs while waiting on all boards to respond
output [15:0] NprtyErr;	// Number of parity errors detected in the incoming UnDoBuf streams
output [15:0] NoverFlow;
output [15:0] Nclr0, Nclr1, Nclr2, Nclr3, Nclr4, Nclr5, Nclr6, Nclr7;

wire [7:0] UnDoBuf;
reg [3:0] BufClr;
wire [1:0] Tag0, Tag1, Tag2, Tag3, Tag4, Tag5, Tag6, Tag7;
wire [7:0] Signal, parityError;
reg enableOut;

parameter [7:0] Mask = 8'b0000011;

assign UnDoBuf = UnDoBufIn & Mask;

UnDoDecode UnDoDecode0(parityError[0],Signal[0],Tag0,UnDoBuf[0],Clock,Reset,5'd0);
UnDoDecode UnDoDecode1(parityError[1],Signal[1],Tag1,UnDoBuf[1],Clock,Reset,5'd1);
UnDoDecode UnDoDecode2(parityError[2],Signal[2],Tag2,UnDoBuf[2],Clock,Reset,5'd2);
UnDoDecode UnDoDecode3(parityError[3],Signal[3],Tag3,UnDoBuf[3],Clock,Reset,5'd3);
UnDoDecode UnDoDecode4(parityError[4],Signal[4],Tag4,UnDoBuf[4],Clock,Reset,5'd4);
UnDoDecode UnDoDecode5(parityError[5],Signal[5],Tag5,UnDoBuf[5],Clock,Reset,5'd5);
UnDoDecode UnDoDecode6(parityError[6],Signal[6],Tag6,UnDoBuf[6],Clock,Reset,5'd6);
UnDoDecode UnDoDecode7(parityError[7],Signal[7],Tag7,UnDoBuf[7],Clock,Reset,5'd7);

wire [7:0] overFlow, notEmpty;
wire [1:0] TagOut0, TagOut1, TagOut2, TagOut3, TagOut4, TagOut5, TagOut6, TagOut7;
TagFIFO TagFIFO0(overFlow[0], TagOut0, notEmpty[0], enableOut, Tag0, Signal[0], Clock, Reset,3'd0);
TagFIFO TagFIFO1(overFlow[1], TagOut1, notEmpty[1], enableOut, Tag1, Signal[1], Clock, Reset,3'd1);
TagFIFO TagFIFO2(overFlow[2], TagOut2, notEmpty[2], enableOut, Tag2, Signal[2], Clock, Reset,3'd2);
TagFIFO TagFIFO3(overFlow[3], TagOut3, notEmpty[3], enableOut, Tag3, Signal[3], Clock, Reset,3'd3);
TagFIFO TagFIFO4(overFlow[4], TagOut4, notEmpty[4], enableOut, Tag4, Signal[4], Clock, Reset,3'd4);
TagFIFO TagFIFO5(overFlow[5], TagOut5, notEmpty[5], enableOut, Tag5, Signal[5], Clock, Reset,3'd5);
TagFIFO TagFIFO6(overFlow[6], TagOut6, notEmpty[6], enableOut, Tag6, Signal[6], Clock, Reset,3'd6);
TagFIFO TagFIFO7(overFlow[7], TagOut7, notEmpty[7], enableOut, Tag7, Signal[7], Clock, Reset,3'd7);

parameter [3:0] Wait = 4'b0001;		//Wait for one board to send a buffer clear signal (queue not empty)
parameter [3:0] Rest = 4'b0010;		//Wait for all the others to send their signals
parameter [3:0] Paus = 4'b0100;		//Wait one clock cycle
parameter [3:0] Send = 4'b1000;		//Send an all clear signal

reg [3:0] State, NextState;
reg [15:0] NTagErr, NTimeOut, NprtyErr, NoverFlow;
reg [15:0] CntTime;

always @ (State or notEmpty or CntTime or Mask) begin
	case (State)
		Wait: 	begin
					if (notEmpty == 0) NextState = Wait;
					else NextState = Rest;
				end
		Rest:	begin
					if ((notEmpty | ~Mask) == 8'b11111111 || CntTime == 16'b1111111111111111) NextState = Paus;
					else NextState = Rest;
				end
		Paus:	begin
					NextState = Send;
				end
		Send:	begin
					NextState = Wait;
				end
		default: NextState = Wait;
	endcase
end

reg [15:0] Nclr0, Nclr1, Nclr2, Nclr3, Nclr4, Nclr5, Nclr6, Nclr7;
always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
		NTagErr <= 0;
		NTimeOut <= 0;
		NprtyErr <= 0;
		NoverFlow <= 0;
		enableOut <= 1'b0;
		Nclr0 <= 0;
		Nclr1 <= 0;
		Nclr2 <= 0;
		Nclr3 <= 0;
		Nclr4 <= 0;
		Nclr5 <= 0;
		Nclr6 <= 0;
		Nclr7 <= 0;
	end else begin
		if (Signal[0] & Mask[0]) Nclr0 <= Nclr0 + 1;
		if (Signal[1] & Mask[1]) Nclr1 <= Nclr1 + 1;
		if (Signal[2] & Mask[2]) Nclr2 <= Nclr2 + 1;   //These counters do not work for some as-yet mysterious reason
		if (Signal[3] & Mask[3]) Nclr3 <= Nclr3 + 1;
		if (Signal[4] & Mask[4]) Nclr4 <= Nclr4 + 1;
		if (Signal[5] & Mask[5]) Nclr5 <= Nclr5 + 1;
		if (Signal[6] & Mask[6]) Nclr6 <= Nclr6 + 1;
		if (Signal[7] & Mask[7]) Nclr7 <= Nclr7 + 1;
		if (overFlow != 0) NoverFlow <= NoverFlow + 1;
		State <= NextState;
		if (parityError != 0) NprtyErr <= NprtyErr + 1;
		case (State)
			Wait:	begin
						BufClr <= 1'b0;
						CntTime <= 0;
						if (notEmpty != 0) $display("%g\t FEbufferMgr: first BufClr signal, notEmpty=%b, Mask=%b.",$time,notEmpty,Mask);
					end
			Rest:	begin
						if (NextState == Paus) begin
							enableOut <= 1'b1;
						end
						CntTime <= CntTime + 1;
						if (CntTime == 16'b1111111111111111) begin
							NTimeOut <= NTimeOut + 1;
							$display("%g\t FEbufferMgr: TimeOut occurance %d.",$time,NTimeOut);
						end
//						$display("%g\t FEbufferMgr: waiting.  notEmpty=%b.",$time,notEmpty);
					end
			Paus:	begin
						enableOut <= 1'b0;
					end
			Send:	begin
//						$display("%g\t FEbufferMgr:  sending out a buffer clear signal for tag %d",$time,TagOut0);
						BufClr[TagOut0] <= 1'b1;
						//The following check assumes that the first tracker layer is always present
						if ((Mask[1] && TagOut1 != TagOut0) || (Mask[2] && TagOut2 != TagOut0) || (Mask[3] && TagOut3 != TagOut0) || (Mask[4] && TagOut4 != TagOut0) || (Mask[5] && TagOut5 != TagOut0) || (Mask[6] && TagOut6 != TagOut0) || (Mask[7] && TagOut7 != TagOut0)) begin
							NTagErr <= NTagErr + 1;
//							$display("%g\t FEbufferMgr: Tag error occurance %d.",$time,NTagErr);
						end
					end
		endcase
	end
end

endmodule
