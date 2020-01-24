//64-bit shift register with parallel in, serial out

module shift64_pi_so (clk, load, pi, so);
input clk;       //Clock to shift
input load;      //Signal to load parallel in (synchronous)
input [63:0] pi; //Parallel input data
output so;       //Serial output data

wire clk, load;
wire [63:0] pi;
wire so;

reg [63:0] tmp;  //Internal register holding data to shift out

always @ (posedge clk)
begin
  if (load)
    tmp <= pi;
  else
    tmp <= {tmp[62:0], 1'b1};
//  $display("%g\t  %h     %b", $time, tmp, so);
end

assign so = tmp[63];

endmodule
