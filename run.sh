#!/bin/bash

openocd -f board/stm32l4discovery.cfg &
OPENOCD_PID=$!
echo $OPENOCD_PID

cleanup () {
    echo "Killing openocd process..."
    kill -9 $OPENOCD_PID
    wait $OPENOCD_PID 2>/dev/null
}


trap cleanup EXIT INT

sleep .5
echo -e "halt\nflash write_image erase target/thumbv7em-none-eabi/debug/rusty-stm32-shell\nreset\nexit\n" | nc localhost 4444

#telnet localhost 4444
#sleep 0.5
#echo "halt"
#echo "flash write_image erase target/thumbv7em-none-eabi/debug/rusty-stm32-shell"
#echo "reset"
