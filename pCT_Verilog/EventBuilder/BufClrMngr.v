// Keep track of the front-end buffer occupancy of the pCT tracker ASICs, and let
// the data acquisition know when a free buffer is available.
// R. Johnson,  March 20, 2013
// Modified 4/13/2014 to make it function properly when MaskIn is not all 1's
module BufClrMngr(MaskIn, NErr, Blip, BufClr, UnDoBufIn, Trigger, Clock, Reset, NFull);

input Clock;
input Reset;
input [7:0] MaskIn;		// Which tracker layers are in the readout
input [7:0] UnDoBufIn;	// Serial stream from each of the 8 tracker boards
input Trigger;			// Trigger condition indicating a new event is taken
output [11:0] BufClr;	// Set true for each tracker FPGA if it has a clear buffer
output [15:0] NErr;		// Number of errors detected in the incoming UnDoBuf streams
output [2:0] NFull;		// Number of front-end buffers full in the worst case at any given time
output [11:0] Blip;		// Signal for each FPGA each time a buffer clear is detected

wire [7:0] UnDoBuf;
wire [11:0] BufClr;
wire [7:0] Error;
reg [15:0] NErr;
reg [2:0] NFull;
wire [2:0] NFull0, NFull1, NFull2, NFull3, NFull4, NFull5, NFull6, NFull7;

assign UnDoBuf = UnDoBufIn & MaskIn;
wire [11:0] MaskOut;
assign MaskOut[3:0] = MaskIn[3:0];
assign MaskOut[4] = MaskIn[4];
assign MaskOut[5] = MaskIn[4];
assign MaskOut[6] = MaskIn[5];
assign MaskOut[7] = MaskIn[5];
assign MaskOut[8] = MaskIn[6];
assign MaskOut[9] = MaskIn[6];
assign MaskOut[10] = MaskIn[7];
assign MaskOut[11] = MaskIn[7];
wire [11:0] BufClrOut;
assign BufClrOut = (BufClr | ~MaskOut);

BufClrDecodeV BufClrDecode0(Blip[0],UnDoBuf[0],Trigger & MaskIn[0],BufClr[0],NFull0,Error[0],Clock,Reset,3'h0);
BufClrDecodeV BufClrDecode1(Blip[1],UnDoBuf[2],Trigger & MaskIn[1],BufClr[1],NFull1,Error[1],Clock,Reset,3'h1);
BufClrDecodeV BufClrDecode2(Blip[2],UnDoBuf[5],Trigger & MaskIn[2],BufClr[2],NFull2,Error[2],Clock,Reset,3'h2);
BufClrDecodeV BufClrDecode3(Blip[3],UnDoBuf[7],Trigger & MaskIn[3],BufClr[3],NFull3,Error[3],Clock,Reset,3'h3);
BufClrDecodeT BufClrDecode4(Blip[4],Blip[5],UnDoBuf[1],Trigger & MaskIn[4],BufClr[4],BufClr[5],NFull4,Error[4],Clock,Reset,3'h4);
BufClrDecodeT BufClrDecode5(Blip[6],Blip[7],UnDoBuf[3],Trigger & MaskIn[5],BufClr[6],BufClr[7],NFull5,Error[5],Clock,Reset,3'h5);
BufClrDecodeT BufClrDecode6(Blip[8],Blip[9],UnDoBuf[4],Trigger & MaskIn[6],BufClr[8],BufClr[9],NFull6,Error[6],Clock,Reset,3'h6);
BufClrDecodeT BufClrDecode7(Blip[10],Blip[11],UnDoBuf[6],Trigger & MaskIn[7],BufClr[10],BufClr[11],NFull7,Error[7],Clock,Reset,3'h7);
always @ (BufClr) begin
	$display("%g\t BufClrMngr: changing BufClr status to %b; NFull=%d %d %d %d %d %d %d %d, NFullMax=%d, NErr=%d",$time,BufClr,NFull0,NFull1,NFull2,NFull3,NFull4,NFull5,NFull6,NFull7,NFull,NErr);
end
always @ (Error) begin
	$display("%g\t BufClrMngr: error flags set to %b",$time,Error);
end

always @ (posedge Clock) begin
	if (Reset) begin
		NErr <= 0;
	end else begin
		if (Error != 0) NErr <= NErr + 1;
	end
end

//The following is a lot of logic just to provide monitoring information on the worst case fill factor.
always @ (NFull0 or NFull1 or NFull2 or NFull3 or NFull4 or NFull5 or NFull6 or NFull7) begin
	if (NFull7 > NFull0 && NFull7 > NFull1 && NFull7 > NFull2 && NFull7 > NFull3 && NFull7 > NFull4 && NFull7 > NFull5 && NFull7 > NFull6) begin
		NFull = NFull7;
	end else if (NFull6 > NFull0 && NFull6 > NFull1 && NFull6 > NFull2 && NFull6 > NFull3 && NFull6 > NFull4 && NFull6 > NFull5) begin
		NFull = NFull6;
	end else if (NFull5 > NFull0 && NFull5 > NFull1 && NFull5 > NFull2 && NFull5 > NFull3 && NFull5 > NFull4) begin
		NFull = NFull5; 
	end else if (NFull4 > NFull0 && NFull4 > NFull1 && NFull4 > NFull2 && NFull4 > NFull3) begin
		NFull = NFull4;
	end else if (NFull3 > NFull0 && NFull3 > NFull1 && NFull3 > NFull2) begin
		NFull = NFull3;
	end else if (NFull2 > NFull0 && NFull2 > NFull1) begin
		NFull = NFull2;
	end else if (NFull1 > NFull0) begin
		NFull=NFull1;
	end else begin
		NFull=NFull0;
	end
end

endmodule

module BufClrDecodeV(Blip,Stream,Trigger,BufClr,NFull,Error,Clock,Reset,Layer);

input Clock;
input Reset;
input Stream;		//Serial stream from a V board
input Trigger;		//True for 1 clock cycle for each accepted trigger
output BufClr;		//True if this board has a buffer available
output [2:0] NFull;		//Number of front-end buffers full
output Error;		//True for 1 clock cycle on an error condition
output Blip;		//Signal each time a buffer clear is received
input [2:0] Layer;

reg [2:0] NFull;
reg Error, Blip;
assign BufClr = (NFull < 4);

parameter [3:0] Wait=4'b0001;		//Wait for a start bit
parameter [3:0] Scnd=4'b0010;		//Get the second bit
parameter [3:0] Thrd=4'b0100;		//Get the third bit and send the BufClr signal
parameter [3:0] Paus=4'b1000;		//Skip one clock before looking for the next signal

reg [3:0] State, NextState;

always @ (State or Stream) begin
	case (State)
		Wait:	begin
					if (Stream) NextState = Scnd;
					else NextState = Wait;
				end
		Scnd:	begin
					NextState = Thrd;
				end
		Thrd:	begin
					NextState = Paus;
				end
		Paus:	begin
					NextState = Wait;
				end
		default:	begin
						NextState = Wait;
					end
	endcase
end

reg raisError, Bit2;
always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
		NFull <= 0;
		Blip <= 1'b0;
	end else begin
		State <= NextState;
		case (State)
			Wait:	begin
						if (Trigger) begin
							NFull <= NFull + 1;
							$display("%g\t BufclrDecodeV %d: incrementing buffer occupancy to %d",$time,Layer,NFull+1);
							if (NFull == 4) begin
								raisError <= 1'b1;  //Trigger should be disabled if NFull=4
								$display("%g\t BufClrDecodeV error-1 %d: trigger not disabled! NFull=%d.",$time,Layer,NFull);
							end else raisError <= 1'b0;
						end else begin
							raisError <= 1'b0;
						end
						Error <= 1'b0;
					end
			Scnd:	begin
						Bit2 <= Stream;
						if (Trigger) begin
							NFull <= NFull + 1;
							$display("%g\t BufclrDecodeV %d: incrementing buffer occupancy to %d",$time,Layer,NFull+1);
							if (NFull == 4) begin
								raisError <= 1'b1;  //Something is off in the counting
								$display("%g\t BufClrDecodeV error-2 %d: trigger not disabled! NFull=%d.",$time,Layer,NFull);
							end
						end
					end
			Thrd:	begin
						if (Bit2 & !Stream) begin
							Blip <= 1'b1;
							if (!Trigger) begin
								NFull <= NFull - 1;
								$display("%g\t BufclrDecodeV %d: decrementing buffer occupancy to %d",$time,Layer,NFull-1);
								if ((Bit2 & !Stream) && (NFull == 0)) begin
									raisError <= 1'b1;
									$display("%g\t BufClrDecodeV error-3 %d: queue underflow! NFull=%d.",$time,Layer,NFull);
								end
							end
						end else begin		//Wrong bit pattern
							if (Trigger) begin
								NFull <= NFull + 1;
								$display("%g\t BufclrDecodeV %d: incrementing buffer occupancy to %d",$time,Layer,NFull+1);
							end
							raisError <= 1'b1;  
							$display("%g\t BufClrDecodeV error-4 %d: wrong bit pattern! NFull=%d.",$time,Layer,NFull);
						end
					end
			Paus:	begin
						Blip <= 1'b0;
						if (raisError) Error <= 1'b1;
						if (Trigger) begin
							NFull <= NFull + 1;
							$display("%g\t BufclrDecodeV %d: incrementing buffer occupancy to %d",$time,Layer,NFull+1);
							if (!raisError && (NFull == 4)) begin
								Error <= 1'b1;
								$display("%g\t BufClrDecodeV error-5 %d: trigger not disabled! NFull=%d.",$time,Layer,NFull);
							end
						end
					end
		endcase
	end
end
endmodule

module BufClrDecodeT(BlipEven,BlipOdd,Stream,Trigger,BufClrEven,BufClrOdd,NFullmax,Error,Clock,Reset,Layer);

input Clock;
input Reset;
input Stream;		//Serial stream from a T board
input Trigger;		//True for 1 clock cycle for each accepted trigger
output BufClrEven;	//True if this board has a buffer available in the even FPGA
output BufClrOdd;	//True if this board has a buffer available in the odd FPGA
output [2:0] NFullmax;	//Number of front-end buffers full (max of the two FPGAs)
output Error;		//True for 1 clock cycle on an error condition
output BlipEven;
output BlipOdd;
input [2:0] Layer;

reg Error, BlipEven, BlipOdd;
reg [2:0] NFullmax, NFullEven, NFullOdd;
assign BufClrEven = (NFullEven < 4);
assign BufClrOdd = (NFullOdd < 4);

always @ (NFullEven or NFullOdd) begin
	if (NFullEven > NFullOdd) begin
		NFullmax = NFullEven;
	end else begin
		NFullmax = NFullOdd;
	end
end

parameter [3:0] Wait=4'b0001;		//Wait for a start bit
parameter [3:0] Scnd=4'b0010;		//Get the second bit
parameter [3:0] Thrd=4'b0100;		//Get the third bit and send the BufClr signal
parameter [3:0] Paus=4'b1000;		//Skip one clock before looking for the next signal

reg [3:0] State, NextState;

always @ (State or Stream) begin
	case (State)
		Wait:	begin
					if (Stream) NextState = Scnd;
					else NextState = Wait;
				end
		Scnd:	begin
					NextState = Thrd;
				end
		Thrd:	begin
					NextState = Paus;
				end
		Paus:	begin
					NextState = Wait;
				end
		default:	begin
						NextState = Wait;
					end
	endcase
end

reg raisError, Bit2;
always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
		NFullEven <= 0;
		NFullOdd <= 0;
		BlipEven <= 1'b0;
		BlipOdd <= 1'b0;
	end else begin
		State <= NextState;
		if ((State != Wait || Trigger) && Layer == 7) begin
			$display("%g\t BufClrDecodeT %d: State=%b Trigger=%b Stream=%b Bit2=%b NFullEven=%d NFullOdd=%d",$time,Layer,State,Trigger,Stream,Bit2,NFullEven,NFullOdd);
		end
		case (State)
			Wait:	begin
						if (Trigger) begin
							NFullEven <= NFullEven + 1;
							NFullOdd <= NFullOdd + 1;
							if (NFullEven == 4 || NFullOdd == 4) begin
								raisError <= 1'b1;  //Trigger should be disabled if NFull=4
								$display("%g\t BufClrDecodeT error-1 %d: trigger not disabled! NFull=%d %d.",$time,Layer,NFullEven,NFullOdd);
							end else raisError <= 1'b0;
						end else begin
							raisError <= 1'b0;
						end
						Error <= 1'b0;
					end
			Scnd:	begin
						Bit2 <= Stream;
						if (Trigger) begin
							NFullEven <= NFullEven + 1;
							NFullOdd <= NFullOdd + 1;
							if (NFullEven == 4 || NFullOdd == 4) begin
								raisError <= 1'b1;  //Something is off in the counting
								$display("%g\t BufClrDecodeT error-2 %d: trigger not disabled! NFull=%d %d.",$time,Layer,NFullEven,NFullOdd);
							end
						end
					end
			Thrd:	begin
						if (Bit2 & !Stream) begin
							BlipEven <= 1'b1;
							if (!Trigger) begin
								NFullEven <= NFullEven - 1;
								//$display("%g\t BufClrDecodeT %d, decrementing NFullEven to %d.",$time,Layer,NFullEven-1);
								if (NFullEven == 0) begin
									raisError <= 1'b1;
									$display("%g\t BufClrDecodeT error-7 %d: underflow! NFull=%d %d.",$time,Layer,NFullEven,NFullOdd);
								end
							end else begin
								NFullOdd <= NFullOdd + 1;
								if (NFullOdd == 4) begin
									raisError <= 1'b1;
									$display("%g\t BufClrDecodeT error-5 %d: trigger not disabled! NFull=%d %d.",$time,Layer,NFullEven,NFullOdd);
								end
							end
						end else if (!Bit2 & Stream) begin
							BlipOdd <= 1'b1;
							if (!Trigger) begin
								NFullOdd <= NFullOdd - 1;
								//$display("%g\t BufClrDecodeT %d, decrementing NFullOdd to %d.",$time,Layer,NFullOdd-1);
								if (NFullOdd == 0) begin
									raisError <= 1'b1;
									$display("%g\t BufClrDecodeT error-8 %d: underflow! NFull=%d %d.",$time,Layer,NFullEven,NFullOdd);
								end
							end else begin
								NFullEven <= NFullEven + 1;
								if (NFullEven == 4) begin 
									raisError <= 1'b1;
									$display("%g\t BufClrDecodeT error-4 %d: trigger not disabled! NFull=%d %d.",$time,Layer,NFullEven,NFullOdd);
								end
							end
						end else begin    //If we go here, then we have trouble. . .
							raisError <= 1'b1;  //Bad code.  Parity error?
							$display("%g\t BufClrDecodeT error-3 %d: Bad code=%b%b. Parity error? NFull=%d %d.",$time,Layer,Bit2,Stream,NFullEven,NFullOdd);
							if (Trigger) begin
								NFullEven <= NFullEven + 1;
								NFullOdd <= NFullOdd + 1;
								if (NFullEven == 4 || NFullOdd == 4) begin
									Error <= 1'b1;
									$display("%g\t BufClrDecodeT error-9 %d: trigger not disabled! NFull=%d %d.",$time,Layer,NFullEven,NFullOdd);
								end
							end
						end
					end
			Paus:	begin
						BlipEven <= 1'b0;
						BlipOdd <= 1'b0;
						if (raisError) Error <= 1'b1;
						if (Trigger) begin
							NFullEven <= NFullEven + 1;
							NFullOdd <= NFullOdd + 1;
							if (!raisError && (NFullEven == 4 || NFullOdd == 4)) begin
								Error <= 1'b1;
								$display("%g\t BufClrDecodeT error-6 %d: trigger not disabled! NFull=%d %d.",$time,Layer,NFullEven,NFullOdd);
							end
						end
					end
		endcase
	end
end
endmodule
