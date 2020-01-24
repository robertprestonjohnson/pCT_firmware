//Stretch and/or delay the fast-OR trigger pulses receive from the front-end boards before making a coincidence
//R. Johnson, May 2013
//August 2, 2013, added trigger delay
//March 30, 2014, removed unintended delays
//October 9, 2014, avoid excess pulse length when the delay is set to zero
//January 27, 2016, fixed bug for zero trigger delay
module TrgStretch(Clock,Reset,TReqIn,TrgPls,TrgLen,TrgDly);
input Clock;
input Reset;
input TReqIn;
input [7:0] TrgLen;   //Additional pulse length after aligning with clock.  Should be 2 or greater.
input [7:0] TrgDly;   //Additional delay of rising edge after aligning with clock.
output TrgPls;

reg TrgPls;

parameter [4:0] TR01=5'b00001;  //Look for the start of the trigger signal
parameter [4:0] TR05=5'b00010;  //Delay the trigger
parameter [4:0] TR02=5'b00100;  //Output a 1-clock long trigger
parameter [4:0] TR03=5'b01000;  //Wait before looking for another trigger
parameter [4:0] TR04=5'b10000;  //Wait for TReqIn to go down
reg [4:0] StateTR, NextStateTR;
reg [7:0] CtrTR;

always @ (StateTR or TReqIn or CtrTR or TrgDly or TrgLen) begin
	case (StateTR)
		TR01: begin
					if (TReqIn) begin
						if (TrgDly<2) NextStateTR = TR02;
						else NextStateTR = TR05;
					end else NextStateTR = TR01;
				end
		TR05:	begin
					if (CtrTR==TrgDly-2) NextStateTR = TR02;
					else NextStateTR = TR05;
				end
		TR02:	begin
					if (TrgLen<2 || (TrgLen==2 && TrgDly==0)) NextStateTR = TR04;
					else NextStateTR = TR03;
				end
		TR03:	begin
					if (CtrTR < TrgLen-2) NextStateTR = TR03;
					else NextStateTR = TR04;
				end
		TR04:	begin
					if (!TReqIn) NextStateTR = TR01;
					else NextStateTR = TR04;
				end
		default: NextStateTR = TR01;
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		StateTR <= TR01;
		TrgPls <= 1'b0;
	end else begin
		StateTR <= NextStateTR;
		case (StateTR)
			TR01:   begin
						if (TrgDly==0 && TReqIn) begin
							TrgPls <= 1'b1;
						end else begin
							TrgPls <= 1'b0;
						end
						CtrTR <= 0;
					end
			TR05:	begin
						CtrTR <= CtrTR + 1;
					end
			TR02:	begin
						TrgPls <= 1'b1;
						CtrTR <= 0;
					end
			TR03:	begin
						CtrTR <= CtrTR + 1;
					end
			TR04:   begin
						TrgPls <= 1'b0;
					end
		endcase
	end
end

endmodule