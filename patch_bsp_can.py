with open("User/bsp_can.c", "r") as f:
    text = f.read()

old_str = """  /*ҵյķ䣬ݷͳȥ*/
uint32_t tx_mailbox;
uint32_t wait_timeout = 0xFFFF;
while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 && wait_timeout > 0) wait_timeout--;
HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
return 0;"""

new_str = """  /* find empty mailbox */
uint32_t tx_mailbox;
uint32_t wait_timeout = 0x0FFFFF;
while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 && wait_timeout > 0) wait_timeout--;

if (wait_timeout > 0) {
HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
return 1;
} else {
return 0;
}"""

if old_str in text:
    print("Match found!")
else:
    print("Match not found!")
