# NOT QUALIFIED — do not enable for hardware
Current explicit overlay is an experiment, not the normal runtime.
manager_rpc_ordered_01 exits with a frozen lifecycle RPC, but nav_restart_ordered_01
still has SIGSEGV during shutdown. nav_restart_ordered_gdb_01 confirms fatal signals.
All functional goal/restart checks are positive; shutdown acceptance fails.
Current investigation: RMW wait set lifetime during cancellation and concurrent
bond cleanup. The backtrace is evidence of a crash site, not a fully proved root cause.
No further candidate experiments are running. Work is prioritizing nominal localization
and the hardware adapter; this defect remains an open production acceptance requirement.
