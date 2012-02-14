Linux for the Chumby Hackers Board
==================================

This is a fork of the Linux kernel which incorporates
the modifications to support the Chumby Hackers Board.
The official Chumby release supports only kernel 2.6.28.
This fork will provide later kernel versions for the CHB.

```
git clone http://opensource.freescale.com/pub/scm/imx/linux-2.6-imx.git
git remote add --mirror github git@github.com:clearwater/chumby-linux.git
```

For each of the branches we want to copy

```
B=imx_2.6.31
git checkout -t origin/$B
git push github $B
```

Now create a new branch for the Chumby code.
I determined (painfully) that the following commit
was the closest to the sources provided by Chumby, so
this is where we diverge.

```
git checkout ec75a15b9fab3e16e456838d79420a3cfab06573
git checkout -b falconwing 
git push github -u falconwing
```

Now replace the kernel files with those from Chumby

```
rm -rf *
tar -zx --strip-components 1 -f ../../linux-2.6.28.mx233-falconwing-1.0.7-053111
.tgz
git add --all
git commit -m "Replace with kernel source from http://files.chumby.com/source/falconwing/build3454/linux-2.6.28.mx233-falconwing-1.0.7-053
111.tgz"
```
