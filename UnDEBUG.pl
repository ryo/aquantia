#!/usr/local/bin/perl

use strict;
use warnings;

my $in_xxx = 0;
my $discard_blankline = 0;
while (<>) {
	if (m,^//COMMENT:,) {
		$discard_blankline = 1;
		next;
	}

	if (m,^//#define\s+XXX_,) {
		$discard_blankline = 1;
		next;
	}

	if (m,^#ifn?def\s+XXX_,) {
		$in_xxx = 1;
	}

	if ($in_xxx && m,^#endif,) {
		$in_xxx = 0;
		$discard_blankline = 1;
		next;
	}

	if ($in_xxx) {
		next;
	}




	if ($discard_blankline && $_ eq "\n") {
		$discard_blankline = 0;
		next;
	}

	$discard_blankline = 0;

	print;


}
