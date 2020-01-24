//Module to generate the command string corresponding to a read command
//This version sends the abreviated command to the tracker board FPGA
//and the corresponding command to the energy digitizer FPGA
//R. Johnson
//October 25, 2013, remove energy detector read commands from here

module GenRead3(Reset, Clock, Busy, Trig, TrigTag, MyBusy, Cmd);
input Reset;           //Hard reset
input Clock;           //System clock
input Busy;			   //Command line is busy when high; must wait
input Trig;            //Signal to start sending the read command
input [1:0] TrigTag;   //Buffer address for the read command
output MyBusy;         //Hold high while the command string is shifting out
output Cmd;            //Output serial command string for the tracker

reg Cmd, MyBusy;
parameter [3:0] Address = 4'b1111;  //Front-end board FPGA wild card address

reg [17:0] CmdOut;     //Temporary shift register for the address and command
reg [4:0] CntT;        //Counter for shifting out command bits
reg [1:0] TagSv;

//Enumeration of states for the read command machine
parameter [3:0] Look = 4'b0001;  //Look for an outgoing trigger
parameter [3:0] Wait = 4'b0010;  //Wait for busy to drop
parameter [3:0] Strt = 4'b0100;  //Send the start bit out
parameter [3:0] Addr = 4'b1000;  //Send the address and command out
reg [3:0] State, NextState;

//State machine for issuing the read command
always @ (State or Trig or CntT or Busy)
begin
  case (State)
    Look: if (Trig) 
			if (!Busy) NextState = Strt;
			else NextState = Wait;
          else
            NextState = Look;
	Wait: if (Busy) NextState = Wait;
		  else NextState = Strt;
    Strt: NextState = Addr;
    Addr: if (CntT == 17) 
            NextState = Look;
          else
            NextState = Addr;
    default: NextState = Look;
  endcase
end

assign Prty = TagSv[1]^TagSv[0];

always @ (posedge Clock)
begin
  if (Reset) begin
    State <= Look;
  end else begin
    State <= NextState;
    case (State)
      Look: begin
              CntT <= 0;
              Cmd <= 1'b0;
              MyBusy <= 1'b0;
			  if (Trig) TagSv <= TrigTag;
            end
	  Wait: begin
			  MyBusy <= 1'b1;
			end
      Strt: begin
               Cmd <= 1'b1;                       //Send the start bit for the ASIC read command
               MyBusy <= 1'b1;
			   CmdOut <= {Address,11'b11111110010,Prty,TagSv};   		//ASIC read command
            end
      Addr: begin
              CntT <= CntT + 1;
              Cmd <= CmdOut[17];
              CmdOut <= {CmdOut[16:0], 1'b0};  //Shift out the chip address and command string
            end
    endcase
  end
end

endmodule