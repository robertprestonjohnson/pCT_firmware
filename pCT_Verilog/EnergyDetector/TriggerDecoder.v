//Decode the trigger stream from the event builder
//R. Johnson  August 18, 2013
module TriggerDecoder(Clock, Reset, Trig, Tack, Tag, NPrtyErr);

input Clock;				//System clock
input Reset;				//Active high reset
input Trig;					//Input trigger acknowledge signal
output Tack;				//Trigger signal 1 clock long
output [1:0] Tag;			//Trigger tag
output [15:0] NPrtyErr;		//Number of parity errors detected

wire Clock, Reset, Trig;
reg Tack;
reg [1:0] Tag;
reg [15:0] NPrtyErr;

//Binary encoding of states:
parameter [3:0] IDLE = 4'b0001;   //Look for a start bit
parameter [3:0] CB01 = 4'b0010;   //Catch the first tag bit
parameter [3:0] CB02 = 4'b0100;   //Catch the second tag bit
parameter [3:0] PRTY = 4'b1000;   //Check the parity bit

reg [3:0] State, NextState;

always @ (State or Trig)
begin:COMBINATORIAL_LOGIC
  NextState=0;
  case(State)
    IDLE:if (Trig == 1'b1) begin
           NextState=CB01;
         end else begin
           NextState=IDLE;
         end
    CB01: NextState=CB02;
    CB02: NextState=PRTY;
    default: NextState=IDLE;
  endcase
end

always @ (posedge Clock)
begin:SEQUENTIAL_LOGIC
  if (Reset) begin
    State <= IDLE;
	NPrtyErr <= 0;
  end else begin
    State <= NextState;
    case(State)
      IDLE: Tack <= 1'b0;
      CB01: begin
              Tag[1] <= Trig;
            end 
      CB02: begin
              Tack <= 1'b1;
              Tag[0] <= Trig;
            end 
	  PRTY: begin
				Tack <= 1'b0;
				$display("%g\t TriggerDecoder:  1%b%b.",$time,Tag,Trig);
				if (~(Tag[0]^Tag[1]^Trig)) begin
					NPrtyErr <= NPrtyErr + 1;
					$display("%g\t TriggerDecoder:  Parity error detected in trigger stream.",$time);
				end
			end
    endcase
  end
end

endmodule