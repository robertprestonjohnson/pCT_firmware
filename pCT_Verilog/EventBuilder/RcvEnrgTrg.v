// Receive the trigger primitive, including bit pattern, from an energy detector board.
// R. Johnson  4/15/2016
module RcvEnrgTrg(Clock, Reset, TReqIn, TReqOut, TrgWord, Address);

input Clock;
input Reset;
input TReqIn;
output TReqOut;
output [2:0] TrgWord;
input Address;

reg [2:0] TrgWord;
reg TReqOut;

parameter [2:0] Wait=3'b001;
parameter [2:0] Inpt=3'b010;
parameter [2:0] Done=3'b100;

reg [2:0] State, NextState;
reg [1:0] Cnt;

always @ (State or Cnt or TReqIn) begin
    case (State)
	    Wait: begin
		          if (TReqIn) NextState = Inpt;
				  else NextState = Wait;
		      end
	    Inpt: begin
		          if (Cnt == 2) NextState = Done;
				  else NextState = Inpt;
		      end
		Done: begin
		          NextState = Wait;
		      end
	    default: begin
		             NextState = Wait;
		         end
	endcase
end

always @ (posedge Clock) begin
    if (Reset) begin
	    State <= Wait;
	end else begin
	    State <= NextState;
		if (State != Wait || (TReqIn | TReqOut)) $display("%g\t RcvEnrgTrg %b: State=%b, Cnt=%d, TReqIn=%b, TrgWord=%b, TReqOut=%b",$time,Address,State,Cnt,TReqIn,TrgWord,TReqOut);
		case (State)
		    Wait: begin
			          Cnt <= 0;
					  TReqOut <= 1'b0;
			      end
			Inpt: begin
			          Cnt <= Cnt + 1;
					  TrgWord <= {TrgWord[1:0],TReqIn};
			      end
			Done: begin
			          TReqOut <= 1'b1;
			      end
		endcase
    end
end

endmodule
