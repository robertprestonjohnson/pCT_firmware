// Decode the 4-bit string for the undo-buffer signal
// R. Johnson, 9/4/2013
module UnDoDecode(parityError,Signal,Tag,UnDoBuf,Clock,Reset,Instance);

input [4:0] Instance;	// Specifies the tracker board
input Clock;			// 100 MHz system clock
input Reset;			// System reset
input UnDoBuf;			// Serial input 
output Signal;			// 1-clock long output signal
output [1:0] Tag;		// Decoded trigger tag
output parityError;		// Parity error signal

reg Signal;
reg [1:0] Tag;

parameter [2:0] Wait= 3'b001;	//Wait for the start bit of a signal
parameter [2:0] CkIn= 3'b010;	//Clock in the trigger tag
parameter [2:0] Prty= 3'b100;	//Check the parity bit

reg [2:0] State, NextState;
reg [1:0] Cnt;
reg Parity;
reg parityError;

always @ (State or UnDoBuf or Cnt) begin
	case (State)
		Wait:	begin
					if (UnDoBuf) NextState = CkIn;
					else NextState = Wait;
				end
		CkIn:	begin
					if (Cnt == 1) NextState = Prty;
					else NextState = CkIn;
				end
		Prty:	begin
					NextState = Wait;
				end
		default:	NextState = Wait;
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
	end else begin
		State <= NextState;
		case (State)
			Wait:	begin
						Cnt <= 0;
						Parity <= 1'b1;
						Signal <= 1'b0;
						parityError <= 1'b0;
					end
			CkIn:	begin
						Cnt <= Cnt + 1;
						Tag <= {Tag[0],UnDoBuf};
						Parity <= Parity^UnDoBuf;
					end
			Prty:	begin
						if (Parity^UnDoBuf != 1'b0) begin
							$display("%g\t UnDoDecode %d: parity error, Tag=%b",$time,Instance,Tag);
							parityError <= 1'b1;
						end
						Signal <= 1'b1;
//						$display("%g\t UnDoDecode %d signal: Tag=%d, Parity bit=%b.",$time,Instance,Tag,UnDoBuf);
					end
		endcase
	end
end

endmodule
