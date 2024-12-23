#! /usr/bin/perl 

use strict;
use Term::ANSIColor;

my $cmd  = shift;
my $testname = shift;
if($cmd =~ m/help/) { Usage(); }
elsif($cmd =~ m/check/) { Check(); }
elsif($cmd =~ m/clean/) { Clean(); }
elsif($cmd =~ m/gen_asm/) { GenASMSource(); }
else  { Usage(); }

sub Check {
  my $p_uvmFail = qr/^(UVM_ERROR|UVM_FATAL) (\S+) (@\s*\d+:) (\S+) (\[\S+\]) (.*)$/;
  my $p_errFail = qr/Error|((?<!UVM_)ERROR)/;
  foreach (@ARGV) {
    open my $fh, '+<', $_ or die "Open $_ failed: $!";

    my $match = grep m/$p_uvmFail|$p_errFail/g, <$fh>;

    if($match) {
      print color "bold red";
      print "====FAIL==== $testname\n";
      print $fh "====FAIL==== $testname\n";
      print color "reset";
    }
    else {
      print color "bold green";
      print "====PASS==== $testname\n";
      print $fh "====PASS==== $testname\n";
      print color "reset";
    }
    close $fh;
  }
}

sub Clean {
  my $p_uvmInfo = qr/^(UVM_\S+) (\S+) (@\s*\d+:) (\S+) (\[\S+\]) (.*)$/;
  #                   $1       $2:path $3:time  $4:hier $5:label  $6:info
  foreach (@ARGV) {
    open my $fh, '+<', $_ or die "Open $_ failed: $!";
    my @lines = <$fh>;
    seek($fh, 0, 0);
    truncate($fh, 0);
    foreach my $line (@lines) {
      $line =~ s/$p_uvmInfo/$3 $6/g;
      print $fh $line;
    }
    close $fh;
  }
}


sub Usage {
  die <<EOU;
  Usage:
    perl ./check_test.pl <check|clean> <test_name> <files ...> 
      check: check logs failures
      clean: clean up UVM logs
EOU
}
