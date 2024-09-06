target extended-remote :3333
load
monitor arm semihosting enable
break main
break main.rs:123
break checkpoint/mod.rs:452