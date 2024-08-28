target extended-remote :3333
load
monitor arm semihosting enable
break main
break main.rs:59
break main.rs:74
break checkpoint/mod.rs:452