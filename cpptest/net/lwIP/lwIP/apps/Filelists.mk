# TCPECHO_RAW_FILES: TCP echo server on lwIP Raw API
TCPECHO_RAW_FILES= $(LWIPAPPDIR)/tcpecho_raw/tcpecho_raw.c

# UDPECHO_RAW_FILES: UDP echo server on lwIP Raw API
UDPECHO_RAW_FILES=$(LWIPAPPDIR)/udpecho_raw/udpecho_raw.c

LWIP_NO_SYS_TEST_APPS=$(TCPECHO_RAW_FILES) \
	$(UDPECHO_RAW_FILES)

# TCPECHO__FILES: TCP echo server on lwIP NetConn/Socket
TCPECHO_FILES= $(LWIPAPPDIR)/tcpecho/tcpecho.c

# UDPECHO__FILES: UDP echo server on lwIP NetConn/Socket
UDPECHO_FILES=$(LWIPAPPDIR)/udpecho/udpecho.c

LWIP_RTOS_TEST_APPS=$(TCPECHO_FILES) \
	$(UDPECHO_FILES)

#
#	not clarified 
#
# $(LWIPAPPDIR)/chargen/chargen.c
# $(LWIPAPPDIR)/httpserver/httpserver-netconn.c
# $(LWIPAPPDIR)/netio/netio.c
# $(LWIPAPPDIR)/ping/ping.c
# $(LWIPAPPDIR)/rtp/rtp.c
# $(LWIPAPPDIR)/shell/shell.c
# $(LWIPAPPDIR)/smtp/smtp.c
# $(LWIPAPPDIR)/snmp_private_mib/lwip_prvmib.c
# $(LWIPAPPDIR)/socket_examples/socket_examples.c
# $(LWIPAPPDIR)/tcpecho/tcpecho.c
# $(LWIPAPPDIR)/udpecho/udpecho.c

