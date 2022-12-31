
release:
	perl UnDEBUG.pl if_aq.c > /usr/src/sys/dev/pci/if_aq.c

debug:
	cat if_aq.c > /usr/src/sys/dev/pci/if_aq.c
