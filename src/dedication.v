`default_nettype none

// Silicon art: a dedication engraved on TopMetal1 (see ../macros/dedication.gds).
// Physical-only macro, no logic/no ports -- included in both the RTL and
// gate-level simulation source lists (test/Makefile) since the gate-level
// netlist instantiates it directly.
(* blackbox *) (* keep *)
module dedication ();
endmodule

`default_nettype wire
