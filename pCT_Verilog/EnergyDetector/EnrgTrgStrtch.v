// Latch the energy detector trigger signal and stretch it out for use
// in the coincidence logic.  The signal will be latched on either edge
// of the clock.
// R. Johnson   September 18, 2013
module EnrgTrgStrtch(TrgOut, TrgIn, Length, Clock, Reset);
input Clock;			//100 MHz system clock
input Reset;			//Synchronous reset
input [3:0] Length;		//Length of the desired output signal in clock cycles
input TrgIn;			//Input asynchronous trigger
output TrgOut;			//Output stretched synchronous trigger

wire TrgOut;
reg TrgOutP, TrgOutN;

assign TrgOut = TrgOutP | TrgOutN;

parameter [2:0] Wait= 3'b001;		//Look for the rising edge of the input trigger
parameter [2:0] Strc= 3'b010;		//Stretch the trigger signal according to Length
parameter [2:0] Next= 3'b100;		//Wait for the falling edge of the input trigger

reg [2:0] StateP, NextStateP;
reg [3:0] CntP;

always @ (StateP or TrgIn or CntP or Length) begin
	case(StateP)
		Wait:	begin
					if (TrgIn) NextStateP = Strc;
					else NextStateP = Wait;
				end
		Strc:	begin
					if (CntP == Length) NextStateP = Next;
					else NextStateP = Strc;
				end
		Next:	begin
					if (!TrgIn) NextStateP = Wait;
					else NextStateP = Next;
				end
		default: NextStateP = Wait;
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		StateP <= Wait;
	end else begin
		StateP <= NextStateP;
		case(StateP)
			Wait:	begin
						TrgOutP <= TrgIn;
						CntP <= 0;
					end
			Strc:	begin
						CntP <= CntP + 1;
					end
			Next:	begin
						TrgOutP <= 1'b0;
					end
		endcase
	end
end

reg [2:0] StateN, NextStateN;
reg [3:0] CntN;

always @ (StateN or TrgIn or CntN or Length) begin
	case(StateN)
		Wait:	begin
					if (TrgIn) NextStateN = Strc;
					else NextStateN = Wait;
				end
		Strc:	begin
					if (CntN == Length) NextStateN = Next;
					else NextStateN = Strc;
				end
		Next:	begin
					if (!TrgIn) NextStateN = Wait;
					else NextStateN = Next;
				end
		default: NextStateN = Wait;
	endcase
end

always @ (negedge Clock) begin
	if (Reset) begin
		StateN <= Wait;
	end else begin
		StateN <= NextStateN;
		case(StateN)
			Wait:	begin
						TrgOutN <= TrgIn;
						CntN <= 0;
					end
			Strc:	begin
						CntN <= CntN + 1;
					end
			Next:	begin
						TrgOutN <= 1'b0;
					end
		endcase
	end
end

endmodule
