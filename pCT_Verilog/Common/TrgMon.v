//Monitor the energy detector and tracker triggers
//R. Johnson, August 2013
module TrgMon(Clock,Reset,TReqIn,Ntriggers);
input Clock;
input Reset;
input TReqIn;
output [31:0] Ntriggers;

reg [31:0] Ntriggers;

parameter [2:0] TR01=3'b001;  //Look for the start of the trigger signal
parameter [2:0] TR02=3'b010;  //Output a 1-clock long trigger
parameter [2:0] TR03=3'b100;  //Wait before looking for another trigger
reg [2:0] StateTR, NextStateTR;

always @ (StateTR or TReqIn) begin
	case (StateTR)
		TR01: begin
					if (TReqIn) begin
						NextStateTR = TR02;
					end else NextStateTR = TR01;
				end
		TR02:	begin
					NextStateTR = TR03;
				end
		TR03:	begin
					if (!TReqIn) NextStateTR = TR01;
					else NextStateTR = TR03;
				end
		default: NextStateTR = TR01;
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		StateTR <= TR01;
		Ntriggers <= 0;
	end else begin
		StateTR <= NextStateTR;
		case (StateTR)
			TR02:	begin
						Ntriggers <= Ntriggers + 1;
					end
		endcase
	end
end

endmodule