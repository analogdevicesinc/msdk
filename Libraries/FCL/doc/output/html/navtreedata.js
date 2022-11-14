var NAVTREE =
[
  [ "Deeptrust for PCI security architecture", "index.html", [
    [ "Copyright Notice", "index.html#s2", null ],
    [ "Trademarks", "index.html#trademarks", null ],
    [ "Introduction", "index.html#Introduction", null ],
    [ "Security Architecture Presentation", "index.html#Architecture", [
      [ "Primer on Cortex-M security mechanisms", "index.html#intro_sec", null ],
      [ "Hypervisor-based software isolation", "index.html#isolation", [
        [ "Code partitioning: Core firmware and Secure containers (aka \"boxes\")", "index.html#boxes", [
          [ "Box access control list", "index.html#boxacl", null ],
          [ "Box isolation enforcement", "index.html#mech", null ],
          [ "Instantiation of secure boxes and code signing", "index.html#instbox", null ]
        ] ],
        [ "Hypervisor initialization", "index.html#hypervisorinit", null ],
        [ "Context switches", "index.html#ctxswitch", [
          [ "Initial jump to Thread user mode", "index.html#ppppppp", null ],
          [ "Interrupts and exceptions", "index.html#ppppp", null ],
          [ "Cortex-M Privilege escalation", "index.html#contextswitch", null ],
          [ "RPC mechanism", "index.html#rpc", null ]
        ] ],
        [ "Summary of MPU protection effects", "index.html#MPUEffect", null ]
      ] ],
      [ "Chain-of-Trust, firmware integrity and authenticity", "index.html#secureboot", [
        [ "Secure Boot firmware and verification key are in immutable memories with integrity check", "index.html#secbootmem", null ],
        [ "No code loading/injection is possible except through a secure loader", "index.html#codeinject", [
          [ "Signature verification of downloaded code", "index.html#signcode", null ],
          [ "Loading and Update of additional secure boxes", "index.html#updatebox", null ]
        ] ]
      ] ],
      [ "Summary of software items, keys and their protection", "index.html#sum_switems", null ],
      [ "Additional considerations", "index.html#addl_cons", [
        [ "Absence of backdoors", "index.html#nobackdoor", null ],
        [ "Execution from internal memories", "index.html#execintmem", null ],
        [ "Protection of external memories", "index.html#execextmem", null ],
        [ "Early execution the Secure Boot ROM and of the isolation mechanism", "index.html#earlyexec", null ]
      ] ],
      [ "Hardware enforced security", "index.html#security", [
        [ "Cortex-M mechanisms: MPU, privileges, NVIC", "index.html#core", [
          [ "1. The Cortex-M's Memory Protection Unit (MPU)", "index.html#parmpu", null ],
          [ "2. The Cortex-M's 2-level execution model", "index.html#parlevels", null ],
          [ "3. The Cortex-M's NVIC", "index.html#parnvic", null ]
        ] ],
        [ "NVSRAM - Battery backed non volatile RAM", "index.html#nvsram", null ],
        [ "Sensors", "index.html#sensors", null ],
        [ "Read-Only Memory, One-Time programmable memory", "index.html#rom", null ]
      ] ],
      [ "Secure API", "index.html#AccessAPI", null ],
      [ "Maxim Integrated development process", "index.html#codemanagement", [
        [ "Source code control", "index.html#sourcectl", null ],
        [ "Bug tracking", "index.html#bugtrak", null ],
        [ "Code review", "index.html#codereview", null ],
        [ "Developer's guidelines", "index.html#fwguidance", null ],
        [ "Firmware versioning and management", "index.html#fwversion", null ]
      ] ],
      [ "Conclusion", "index.html#Conclusion", [
        [ "Architecture diagram, PCI firmware perimeter", "index.html#archdiagram", null ]
      ] ]
    ] ],
    [ "Security Guidelines", "_p_c_i_g_u_i_d_a_n_c_e.html", [
      [ "Key Management", "_p_c_i_g_u_i_d_a_n_c_e.html#sect_keymanag", [
        [ "Deeptrust for PCI's  keys", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_keys", null ],
        [ "Additional keys", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_addkeys", null ],
        [ "Storing keys inside the MAX325xx", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_keystor", null ],
        [ "Device Provisioning", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_prov", null ]
      ] ],
      [ "Software development", "_p_c_i_g_u_i_d_a_n_c_e.html#sect_fwdev", [
        [ "Software layout", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_fwlayout", null ],
        [ "Adding new boxes", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_newboxes", null ],
        [ "Firmware signature", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_fwsign", null ],
        [ "Software modularity", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_fwmod", null ],
        [ "Secure boxes code review", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_boxrpc", null ],
        [ "Minimal configuration", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_minconf", null ],
        [ "Cryptographic services", "_p_c_i_g_u_i_d_a_n_c_e.html#sub_crypto", null ]
      ] ],
      [ "PIN and cardholder data management", "_p_c_i_g_u_i_d_a_n_c_e.html#sect_PIN", null ],
      [ "Prompt management", "_p_c_i_g_u_i_d_a_n_c_e.html#sect_Prompt", null ],
      [ "Self Tests", "_p_c_i_g_u_i_d_a_n_c_e.html#sect_SelfTest", null ],
      [ "Customer integration", "_p_c_i_g_u_i_d_a_n_c_e.html#sect_integ", null ]
    ] ],
    [ "Release Notes", "_r_e_l_e_a_s_e__n_o_t_e_s.html", [
      [ "Release Notes", "_r_e_l_e_a_s_e__n_o_t_e_s.html#rn", null ]
    ] ],
    [ "References", "_r_e_f_e_r_e_n_c_e_s.html", null ],
    [ "Software API Specification", "modules.html", "modules" ],
    [ "Index of data structures", "annotated.html", "annotated" ]
  ] ]
];

var NAVTREEINDEX =
[
"_p_c_i_g_u_i_d_a_n_c_e.html"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';