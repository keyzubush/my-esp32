import network
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
sta_if.connect("ivanhoe", "realmendonotclick")

import webrepl
webrepl.start()
