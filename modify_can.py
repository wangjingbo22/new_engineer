import re
with open("User/bsp_can.c", "r") as f:
    text = f.read()

text = text.replace(
    "uint32_t tx_mailbox;\n\tHAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);",
    "uint32_t tx_mailbox;\n\twhile(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {}\n\tHAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);"
)

with open("User/bsp_can.c", "w") as f:
    f.write(text)
