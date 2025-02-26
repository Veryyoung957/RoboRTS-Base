uint32 allowed_fire_count  # bit 0-10: 除远程兑换外，哨兵机器人成功兑换的允许发弹量
uint8 remote_fire_exchange_count  # bit 11-14: 哨兵机器人成功远程兑换允许发弹量的次数
uint8 remote_health_exchange_count  # bit 15-18: 哨兵机器人成功远程兑换血量的次数
bool can_confirm_free_revive  # bit 19: 是否可以确认免费复活
bool can_exchange_immediate_revive  # bit 20: 是否可以兑换立即复活
uint16 immediate_revive_cost  # bit 21-30: 兑换立即复活需要的金币数
uint8 reserved  # bit 31: 保留

bool is_out_of_combat  # bit 0: 哨兵当前是否处于脱战状态
uint16 team_17mm_fire_remaining  # bit 1-11: 队伍 17mm 允许发弹量的剩余可兑换数
uint8 reserved_1  # bit 12-15: 保留
