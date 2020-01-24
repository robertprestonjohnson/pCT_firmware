module TriggerReceiver_RTL(
	Clock, //Clock
	Reset, //Active high reset
    Trig,  //Input trigger acknowledge signal
	Tack,  //Trigger signal
	Tag    //Trigger tag
);

// State machine to detect a signal on the Trigger Acknowledge input
// and pull off the 2-bit trigger tag.  Note that two triggers do
// not need to be separated by a zero.  For example, the sequence
// 0100101110111 would generate 4 sequential triggers with tags
// 00, 01, 10, and 11.

input Clock;
input Reset;
input Trig;
output Tack;
output [1:0] Tag;

wire Clock, Reset, Trig;
reg Tack;
reg [1:0] Tag;

//Binary encoding of states:
parameter [1:0] IDLE = 2'b00;   //Look for a start bit
parameter [1:0] CB01 = 2'b01;   //Catch the first tag bit
parameter [1:0] CB02 = 2'b10;   //Catch the second tag bit

reg [1:0] State, NextState;

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
    CB02: NextState=IDLE;
    default: NextState=IDLE;
  endcase
end

always @ (posedge Clock)
begin:SEQUENTIAL_LOGIC
  if (Reset) begin
    State <= IDLE;
    Tack <= 0;
  end else begin
    State <= NextState;
    case(State)
      IDLE: Tack <= 0;
      CB01: begin
              Tack <= 0;
              Tag[1] <= Trig;
            end 
      CB02: begin
              Tack <= 1;
              Tag[0] <= Trig;
            end 
      default: Tack <= 0; 
    endcase
  end
end

endmodule