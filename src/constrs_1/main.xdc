###############################################################################################
## main.xdc for NEXYS A7   ArchLab TOKYO TECH
###############################################################################################

## 100MHz system clock
###############################################################################################
set_property -dict { PACKAGE_PIN E3  IOSTANDARD LVCMOS33} [get_ports { CLK }];
create_clock -add -name sys_clk -period 10.00 [get_ports {CLK}];

## CPU Reset
###############################################################################################
# set_property -dict { PACKAGE_PIN C12 IOSTANDARD LVCMOS33} [get_ports { RST_X_IN }];

## UART (Serial Port)
###############################################################################################
set_property -dict { PACKAGE_PIN D4  IOSTANDARD LVCMOS33} [get_ports { w_txd }];
set_property -dict { PACKAGE_PIN C4  IOSTANDARD LVCMOS33} [get_ports { w_rxd }];

###############################################################################################
create_generated_clock -name mig_in_clk [get_pins m_clkgen0/inst/mmcm_adv_inst/CLKOUT0]
set_clock_groups -asynchronous -group {mig_in_clk}

#create_generated_clock -name core_clk [get_pins mem_ctrl/dram_con/dram/dram/dram_con_witout_cache/clkgen1/inst/mmcm_adv_inst/CLKOUT0]
create_generated_clock -name core_clk [get_pins c/dram_con/dram/dram/dram_con_witout_cache/clkgen1/inst/mmcm_adv_inst/CLKOUT0]
set_clock_groups -asynchronous -group {core_clk}

###############################################################################################
set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports { w_led[0] }];
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33} [get_ports { w_led[1] }];
set_property -dict { PACKAGE_PIN J13 IOSTANDARD LVCMOS33} [get_ports { w_led[2] }];
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports { w_led[3] }];
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports { w_led[4] }];
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33} [get_ports { w_led[5] }];
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33} [get_ports { w_led[6] }];
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33} [get_ports { w_led[7] }];
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33} [get_ports { w_led[8] }];
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS33} [get_ports { w_led[9] }];
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33} [get_ports { w_led[10] }];
set_property -dict { PACKAGE_PIN T16 IOSTANDARD LVCMOS33} [get_ports { w_led[11] }];
set_property -dict { PACKAGE_PIN V15 IOSTANDARD LVCMOS33} [get_ports { w_led[12] }];
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33} [get_ports { w_led[13] }];
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33} [get_ports { w_led[14] }];
set_property -dict { PACKAGE_PIN V11 IOSTANDARD LVCMOS33} [get_ports { w_led[15] }];


###############################################################################################
set_property -dict { PACKAGE_PIN R12 IOSTANDARD LVCMOS33 } [get_ports { w_led1_B }];
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports { w_led1_G }];
set_property -dict { PACKAGE_PIN N15 IOSTANDARD LVCMOS33 } [get_ports { w_led1_R }];
# set_property -dict { PACKAGE_PIN G14 IOSTANDARD LVCMOS33 } [get_ports { w_led2_B }];
# set_property -dict { PACKAGE_PIN R11 IOSTANDARD LVCMOS33 } [get_ports { w_led2_G }];
# set_property -dict { PACKAGE_PIN N16 IOSTANDARD LVCMOS33 } [get_ports { w_led2_R }];

###############################################################################################
set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports { r_sg[6] }]; # segment a
set_property -dict { PACKAGE_PIN R10 IOSTANDARD LVCMOS33} [get_ports { r_sg[5] }]; # segment b
set_property -dict { PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports { r_sg[4] }]; # segment c
set_property -dict { PACKAGE_PIN K13 IOSTANDARD LVCMOS33} [get_ports { r_sg[3] }]; # segment d
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33} [get_ports { r_sg[2] }]; # segment e
set_property -dict { PACKAGE_PIN T11 IOSTANDARD LVCMOS33} [get_ports { r_sg[1] }]; # segment f
set_property -dict { PACKAGE_PIN L18 IOSTANDARD LVCMOS33} [get_ports { r_sg[0] }]; # segment g
set_property -dict { PACKAGE_PIN H15 IOSTANDARD LVCMOS33} [get_ports { r_sg[7] }]; # segment .

set_property -dict { PACKAGE_PIN J17 IOSTANDARD LVCMOS33} [get_ports { r_an[0] }];
set_property -dict { PACKAGE_PIN J18 IOSTANDARD LVCMOS33} [get_ports { r_an[1] }];
set_property -dict { PACKAGE_PIN T9  IOSTANDARD LVCMOS33} [get_ports { r_an[2] }];
set_property -dict { PACKAGE_PIN J14 IOSTANDARD LVCMOS33} [get_ports { r_an[3] }];
set_property -dict { PACKAGE_PIN P14 IOSTANDARD LVCMOS33} [get_ports { r_an[4] }];
set_property -dict { PACKAGE_PIN T14 IOSTANDARD LVCMOS33} [get_ports { r_an[5] }];
set_property -dict { PACKAGE_PIN K2  IOSTANDARD LVCMOS33} [get_ports { r_an[6] }];
set_property -dict { PACKAGE_PIN U13 IOSTANDARD LVCMOS33} [get_ports { r_an[7] }];

###############################################################################################
set_property -dict { PACKAGE_PIN M18 IOSTANDARD LVCMOS33} [get_ports { w_btnu }];
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33} [get_ports { w_btnd }];
set_property -dict { PACKAGE_PIN P17 IOSTANDARD LVCMOS33} [get_ports { w_btnl }];
set_property -dict { PACKAGE_PIN M17 IOSTANDARD LVCMOS33} [get_ports { w_btnr }];
set_property -dict { PACKAGE_PIN N17 IOSTANDARD LVCMOS33} [get_ports { w_btnc }];
###############################################################################################
# set_property -dict { PACKAGE_PIN J15 IOSTANDARD LVCMOS33 } [get_ports { w_sw[0] }];
# set_property -dict { PACKAGE_PIN L16 IOSTANDARD LVCMOS33 } [get_ports { w_sw[1] }];
# set_property -dict { PACKAGE_PIN M13 IOSTANDARD LVCMOS33 } [get_ports { w_sw[2] }];
# set_property -dict { PACKAGE_PIN R15 IOSTANDARD LVCMOS33 } [get_ports { w_sw[3] }];
# set_property -dict { PACKAGE_PIN R17 IOSTANDARD LVCMOS33 } [get_ports { w_sw[4] }];
# set_property -dict { PACKAGE_PIN T18 IOSTANDARD LVCMOS33 } [get_ports { w_sw[5] }];
# set_property -dict { PACKAGE_PIN U18 IOSTANDARD LVCMOS33 } [get_ports { w_sw[6] }];
# set_property -dict { PACKAGE_PIN R13 IOSTANDARD LVCMOS33 } [get_ports { w_sw[7] }];
# set_property -dict { PACKAGE_PIN T8  IOSTANDARD LVCMOS18 } [get_ports { w_sw[8] }];
# set_property -dict { PACKAGE_PIN U8  IOSTANDARD LVCMOS18 } [get_ports { w_sw[9] }];
# set_property -dict { PACKAGE_PIN R16 IOSTANDARD LVCMOS33 } [get_ports { w_sw[10] }];
# set_property -dict { PACKAGE_PIN T13 IOSTANDARD LVCMOS33 } [get_ports { w_sw[11] }];
# set_property -dict { PACKAGE_PIN H6  IOSTANDARD LVCMOS33 } [get_ports { w_sw[12] }];
# set_property -dict { PACKAGE_PIN U12 IOSTANDARD LVCMOS33 } [get_ports { w_sw[13] }];
# set_property -dict { PACKAGE_PIN U11 IOSTANDARD LVCMOS33 } [get_ports { w_sw[14] }];
# set_property -dict { PACKAGE_PIN V10 IOSTANDARD LVCMOS33 } [get_ports { w_sw[15] }];
###############################################################################################

## Ethernet PHY

set_property -dict { PACKAGE_PIN A9 IOSTANDARD LVCMOS33} [get_ports { w_mdio_phy }];
set_property -dict { PACKAGE_PIN C9 IOSTANDARD LVCMOS33} [get_ports { r_mdc_phy }];
set_property -dict { PACKAGE_PIN B3  IOSTANDARD LVCMOS33} [get_ports { r_rstn_phy }];
set_property -dict { PACKAGE_PIN D9 IOSTANDARD LVCMOS33} [get_ports { w_crs_dv_phy}];
set_property -dict { PACKAGE_PIN A10 IOSTANDARD LVCMOS33} [get_ports { w_txd_phy[0] }];
set_property -dict { PACKAGE_PIN A8 IOSTANDARD LVCMOS33} [get_ports { w_txd_phy[1] }];
set_property -dict { PACKAGE_PIN B9  IOSTANDARD LVCMOS33} [get_ports { w_txen_phy }];
set_property -dict { PACKAGE_PIN C11 IOSTANDARD LVCMOS33} [get_ports { w_rxd_phy[0] }];
set_property -dict { PACKAGE_PIN D10 IOSTANDARD LVCMOS33} [get_ports { w_rxd_phy[1] }];
set_property -dict { PACKAGE_PIN C10  IOSTANDARD LVCMOS33} [get_ports { w_rxerr_phy }];
set_property -dict { PACKAGE_PIN D5 IOSTANDARD LVCMOS33} [get_ports { w_clkin_phy }];

###############################################################################################

##Micro SD Connector

set_property -dict { PACKAGE_PIN E2    IOSTANDARD LVCMOS33 } [get_ports { sd_rst }]; #IO_L14P_T2_SRCC_35 Sch=sd_reset
set_property -dict { PACKAGE_PIN A1    IOSTANDARD LVCMOS33 } [get_ports { sd_cd }]; #IO_L9N_T1_DQS_AD7N_35 Sch=sd_cd
set_property -dict { PACKAGE_PIN B1    IOSTANDARD LVCMOS33 } [get_ports { sd_sclk }]; #IO_L9P_T1_DQS_AD7P_35 Sch=sd_sck
set_property -dict { PACKAGE_PIN C1    IOSTANDARD LVCMOS33 } [get_ports { sd_cmd }]; #IO_L16N_T2_35 Sch=sd_cmd
set_property -dict { PACKAGE_PIN C2    IOSTANDARD LVCMOS33 } [get_ports { sd_dat[0] }]; #IO_L16P_T2_35 Sch=sd_dat[0]
set_property -dict { PACKAGE_PIN E1    IOSTANDARD LVCMOS33 } [get_ports { sd_dat[1] }]; #IO_L18N_T2_35 Sch=sd_dat[1]
set_property -dict { PACKAGE_PIN F1    IOSTANDARD LVCMOS33 } [get_ports { sd_dat[2] }]; #IO_L18P_T2_35 Sch=sd_dat[2]
set_property -dict { PACKAGE_PIN D2    IOSTANDARD LVCMOS33 } [get_ports { sd_dat[3] }]; #IO_L14N_T2_SRCC_35 Sch=sd_dat[3]
###############################################################################################

## VGA Connector

set_property -dict {PACKAGE_PIN A3 IOSTANDARD LVCMOS33} [get_ports {vga_red[0]}]
set_property -dict {PACKAGE_PIN B4 IOSTANDARD LVCMOS33} [get_ports {vga_red[1]}]
set_property -dict {PACKAGE_PIN C5 IOSTANDARD LVCMOS33} [get_ports {vga_red[2]}]
set_property -dict {PACKAGE_PIN A4 IOSTANDARD LVCMOS33} [get_ports {vga_red[3]}]

set_property -dict {PACKAGE_PIN C6 IOSTANDARD LVCMOS33} [get_ports {vga_green[0]}]
set_property -dict {PACKAGE_PIN A5 IOSTANDARD LVCMOS33} [get_ports {vga_green[1]}]
set_property -dict {PACKAGE_PIN B6 IOSTANDARD LVCMOS33} [get_ports {vga_green[2]}]
set_property -dict {PACKAGE_PIN A6 IOSTANDARD LVCMOS33} [get_ports {vga_green[3]}]

set_property -dict {PACKAGE_PIN B7 IOSTANDARD LVCMOS33} [get_ports {vga_blue[0]}]
set_property -dict {PACKAGE_PIN C7 IOSTANDARD LVCMOS33} [get_ports {vga_blue[1]}]
set_property -dict {PACKAGE_PIN D7 IOSTANDARD LVCMOS33} [get_ports {vga_blue[2]}]
set_property -dict {PACKAGE_PIN D8 IOSTANDARD LVCMOS33} [get_ports {vga_blue[3]}]

set_property -dict {PACKAGE_PIN B11 IOSTANDARD LVCMOS33} [get_ports vga_h_sync]
set_property -dict {PACKAGE_PIN B12 IOSTANDARD LVCMOS33} [get_ports vga_v_sync]

#####################################################################################################

##USB HID (PS/2)

set_property -dict { PACKAGE_PIN F4    IOSTANDARD LVCMOS33 } [get_ports { usb_ps2_clk }]; #IO_L13P_T2_MRCC_35 Sch=ps2_clk
set_property -dict { PACKAGE_PIN B2    IOSTANDARD LVCMOS33 } [get_ports { usb_ps2_data }]; #IO_L10N_T1_AD15N_35 Sch=ps2_data

##Pmod Header JB7, JB9
set_property -dict { PACKAGE_PIN E16   IOSTANDARD LVCMOS33 } [get_ports { pmod_ps2_data}]; #IO_L20N_T3_A19_15 Sch=ja[1]
set_property -dict { PACKAGE_PIN G13   IOSTANDARD LVCMOS33 } [get_ports { pmod_ps2_clk}]; #IO_L21P_T3_DQS_15 Sch=ja[3]

##Pmod Header JB10
set_property -dict { PACKAGE_PIN H16   IOSTANDARD LVCMOS33 } [get_ports {ch559_rx}];
