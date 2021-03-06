// XGA_tb.v
// --------------------------------------------------------------
// Tool Version: Quartus 18.1 / ModeSim-Altera 10.5b
// Design      : XGA_tb
// Syntax Ver  : SystemVerilog
// Purpose     : Save each output frame as a BMP file.
//
// -------------------------- Updates --------------------------- 
// | Ver. | Date       | Author      | Comments
// | 1.0  | 03/30/2020 | Victor Gan  | Sucessfully saves BMP
// | 0.9  | 03/28/2020 | Victor Gan  | File Created
// --------------------------------------------------------------

`timescale 1ns /1ps

/*--- VGA PARAMETERS ---*/
// XGA 1024 * 768 @70Hz, 75MHz VGA clock
`define HS 136    // H sync width in pixels
`define HB 144    // H back porch in pixels
`define HD 1024   // H data area in pixels
`define HF 24     // H front porch in pixels
`define VS 6      // V sync width in lines
`define VB 29     // V back porch in lines
`define VD 768    // V data in lines
`define VF 3      // V front porch in lones

module XGA_tb();


   /*--- SYS CLOCK ---*/
   parameter CLK_FREQ_50 = 50_000_000;
   parameter CLK_PERIOD_50 = 1_000_000_000 / CLK_FREQ_50;
   parameter CLK_FREQ_75 = 75_000_000;
   parameter CLK_PERIOD_75 = 1_000_000_000 / CLK_FREQ_75;
   
   reg clk_50;
   always #(CLK_PERIOD_50 / 2) clk_50 = ~clk_50;
   initial clk_50 = 0;
   
   reg clk_75;
   always #(CLK_PERIOD_75 / 2) clk_75 = ~clk_75;
   initial clk_75 = 0;
   

   /*--- DUT VARIABLES ---*/
   // DUT inputs
   reg [9:0] SW;
   reg [3:1] KEY;
   wire KEY0;
   // DUT outputs
   wire [7:0] VGA_R, VGA_G, VGA_B;
   wire VGA_BLANK_N, VGA_SYNC_N, VGA_HS, VGA_VS;
   // VGA_CLK
   wire VGA_CLK;
   // wire clk_75;
   // assign clk_75 = VGA_CLK;
   
   
   /*--- DUT ---*/
   XGA DUT0(
      .CLOCK_50(clk_50),
      .CLOCK_75(clk_75),
      .KEY({KEY,KEY0}),
      .SW(SW),
      .VGA_R(VGA_R),
      .VGA_G(VGA_G),
      .VGA_B(VGA_B),
      .VGA_CLK(VGA_CLK),
      .VGA_BLANK_N(VGA_BLANK_N),
      .VGA_SYNC_N(VGA_SYNC_N),
      .VGA_HS(VGA_HS),
      .VGA_VS(VGA_VS)
   );
   
   
   /*--- TESTBENCH VARIABLES ---*/
   reg rst;
   integer x, y; // x for horizontal, y for vertical
                 // starts from the upper left corner
   integer h_count, v_count;
   integer idx;  // task print_frame
   reg blackdone, reddone, greendone, yellowdone;
   reg bluedone, purpledone, cyandone, whitedone;
   // File IO
   integer black, red, green, yellow, blue, purple, cyan, white;
   // frame buffer
   reg [7:0] frame_b [0:`HD-1][0:`VD-1];
   reg [7:0] frame_g [0:`HD-1][0:`VD-1];
   reg [7:0] frame_r [0:`HD-1][0:`VD-1];
   // buffer initialization
   initial begin
      for (y = 0; y < `VD; y = y + 1) begin
         for (x = 0; x < `HD; x = x + 1) begin
            frame_b[x][y] = 8'd0;
            frame_g[x][y] = 8'd0;
            frame_r[x][y] = 8'd0;
         end
      end
   end
   
   assign KEY0 = ~rst;
   
   
   /*--- BOARD I/O CONTROL ---*/
   initial begin
      SW[9:3] = 7'b0;
      $display("Set SW BLACK");
      SW[2:0] = 3'b000;
      KEY[3:1] = 3'b000;
      
      rst = 0;
      #30 rst = 1;
      #20 rst = 0;
      
      @(posedge blackdone);
      $display("Set SW RED");
      SW[2:0] = 3'b001;
      
      @(posedge reddone);
      $display("Set SW GREEN");
      SW[2:0] = 3'b010;
      
      @(posedge greendone);
      $display("Set SW YELLOW");
      SW[2:0] = 3'b011;
      
      @(posedge yellowdone);
      $display("Set SW BLUE");
      SW[2:0] = 3'b100;
      
      @(posedge bluedone);
      $display("Set SW PURPLE");
      SW[2:0] = 3'b101;
      
      @(posedge purpledone);
      $display("Set SW CYAN");
      SW[2:0] = 3'b110;
      
      @(posedge cyandone);
      $display("Set SW WHITE");
      SW[2:0] = 3'b111;
      
      @(posedge whitedone);
      $display("ALL COLORS DONE!");
      @(posedge clk_75); @(posedge clk_75); @(posedge clk_75);
      $stop;
   end
   
   
   /*--- SCREEN SCANNER ---*/
   initial begin
      h_count = 0;
      v_count = 0;
      blackdone = 0;
      reddone = 0;
      greendone = 0;
      yellowdone = 0;
      bluedone = 0;
      purpledone = 0;
      cyandone = 0;
      whitedone = 0;
      
      // wait for system reset
      @(negedge rst); @(posedge clk_75); @(posedge clk_75);
      
      // Generate BLACK.BMP
      $display("Creating black.bmp");
      black = $fopen("black.bmp", "wb");
      fopen_check(black, "black.bmp");
      print_frame(black, blackdone);
      $fclose(black);
      @(posedge clk_75);
      
      $display("Creating red.bmp");
      red = $fopen("red.bmp", "wb");
      fopen_check(red, "red.bmp");
      print_frame(red, reddone);
      $fclose(red);
      @(posedge clk_75);
      
      $display("Creating green.bmp");
      green = $fopen("green.bmp", "wb");
      fopen_check(green, "green.bmp");
      print_frame(green, greendone);
      $fclose(green);
      @(posedge clk_75);
      
      $display("Creating yellow.bmp");
      yellow = $fopen("yellow.bmp", "wb");
      fopen_check(yellow, "yellow.bmp");
      print_frame(yellow, yellowdone);
      $fclose(yellow);
      @(posedge clk_75);
      
      $display("Creating blue.bmp");
      blue = $fopen("blue.bmp", "wb");
      fopen_check(blue, "blue.bmp");
      print_frame(blue, bluedone);
      $fclose(blue);
      @(posedge clk_75);
      
      $display("Creating purple.bmp");
      purple = $fopen("purple.bmp", "wb");
      fopen_check(purple, "purple.bmp");
      print_frame(purple, purpledone);
      $fclose(purple);
      @(posedge clk_75);
      
      $display("Creating cyan.bmp");
      cyan = $fopen("cyan.bmp", "wb");
      fopen_check(cyan, "cyan.bmp");
      print_frame(cyan, cyandone);
      $fclose(cyan);
      @(posedge clk_75);

      $display("Creating white.bmp");
      white = $fopen("white.bmp", "wb");
      fopen_check(white, "white.bmp");
      print_frame(white, whitedone);
      $fclose(white);
      @(posedge clk_75);
   end
   
   
   /*--- V,H COUNTER & VS,HS EDGE DETECTOR ---*/
   // v_count, h_count
   reg VGA_VS_delay, VGA_HS_delay;
   wire VGA_VS_negedge, VGA_HS_negedge;
   
   always@ (posedge clk_75 or posedge rst) begin
      if (rst) begin
         VGA_VS_delay <= 1;
         VGA_HS_delay <= 1;
         v_count <= 0;
         h_count <= 0;
      end else begin
         VGA_VS_delay <= VGA_VS;
         VGA_HS_delay <= VGA_HS;
         v_count <= VGA_VS_negedge ? 1 : VGA_HS_negedge ? v_count + 1 : v_count;
         h_count <= VGA_HS_negedge ? 1 : h_count + 1;
      end
   end
 
   assign VGA_VS_negedge = VGA_VS_delay & ~VGA_VS;
   assign VGA_HS_negedge = VGA_HS_delay & ~VGA_HS;
   
   
   /*--- BMP TASKS: SystemVerilog Syntax ---*/
   // OPEN FILE
   task fopen_check (input integer file, input string filename);
      if (file) begin
         $write("File <");
         $write(filename);
         $write("> opened succesfully.\n");
      end else begin
         $write("Failed to open file <");
         $write(filename);
         $write(">\n");
         $stop;
      end
   endtask
   
   // BMP HEADER FUNCTION
   task bmp_header (input integer ofile);
      // header
      $fwrite(ofile, "BM"); // signature
      $fwrite(ofile, "%u%u%u",
              32'h00240036,   // FileSize 
              32'd0,          // reserved
              32'h00000036 ); // data offset
      // info header
      $fwrite(ofile, "%u%u%u%u%u%u%u%u%u%u",
              32'h00000028,   // size of info header
              32'h00000400,   // width (pixels)
              32'h00000300,   // height (pixels)
              32'h00180001,   // planes (0x0001) + bits per pixel (0x0018)
              32'd0,          // compression
              32'h00240000,   // image size
              32'd0,          // x ppm
              32'd0,          // y ppm
              32'd0,          // colors used
              32'd0 );        // important colors        
   endtask
   
   
   // Print the whole frame to file
   task print_frame(input integer ofile, output reg done);
      done = 0;
      
      bmp_header(ofile);
      
      // fill frame buffers
      while( !done ) begin
         wait(v_count == (`VS + `VB));
         for (y = 0; y < `VD; y = y + 1) begin
            wait(h_count == (`HS + `HB - 1));
            for (x = 0; x < `HD; x = x + 1) begin
               @(posedge clk_75); #5;
               frame_b[x][y] = VGA_B;
               frame_g[x][y] = VGA_G;
               frame_r[x][y] = VGA_R;
               if (x == 0 && y == 0) begin
                  $display("Start scanning the first pixel: B:%3d G:%3d R:%3d", VGA_B, VGA_G, VGA_R);
               end
            end
            // $display("x:%4d y:%3d -- R:%3d G:%3d B:%3d", x, y, VGA_R, VGA_G, VGA_B);
         end
         @(posedge clk_75);@(posedge clk_75);@(posedge clk_75);
         done = 1;
      end
      
      // dump frame buffer to file
      // BMP scans from the bottom left
      for (y = `VD - 1; y >= 0; y = y - 1) begin
         for (idx = 0; idx < (`HD / 4); idx = idx + 1) begin
            x = idx * 4;
            // deal with ModelSim %u bug:
            // 1) byte littel endian
            // 2) has to send 4 bytes at once
            $fwrite(ofile, "%u", {frame_b[x + 1][y], frame_r[x][y], frame_g[x][y], frame_b[x][y]});
            $fwrite(ofile, "%u", {frame_g[x + 2][y], frame_b[x + 2][y], frame_r[x + 1][y], frame_g[x + 1][y]});
            $fwrite(ofile, "%u", {frame_r[x + 3][y], frame_g[x + 3][y], frame_b[x + 3][y], frame_r[x + 2][y]});
            // $display("writing position (%4d, %3d)", x, y);
         end
      end
   endtask

endmodule


// Simulation Guide
/*
// With Altera_PLL: Gate Level Simulation
vdel -lib gate_work -all
vlib gate_work
vmap work gate_work
vlog simulation/modelsim/XGA.v
vlog -sv XGA_tb.v
vsim -c XGA_tb -L gate_work -L altera_ver -L altera_lnsim_ver -L cyclonev_ver -do "run -a; exit;"

// Testbench Generated VGA_CLK: RTL Simulation
vdel -lib work -all
vlib work
vlog -sv *.v
vsim -c XGA_tb -do "run -a"
*/