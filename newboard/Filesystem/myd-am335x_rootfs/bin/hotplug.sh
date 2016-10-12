#!/bin/sh

MOUNTPOINT=/media/${MDEV}

test ! -c /dev/null && mknod -m 0666 /dev/null c 1 3

case $ACTION in
remove)
	/bin/umount $MOUNTPOINT || true
	rmdir $MOUNTPOINT >/dev/null 2>&1 || true
	;;
*)
	/bin/mkdir -p $MOUNTPOINT > /dev/null 2>&1 || true
    /bin/mount -o async,noatime,nodiratime /dev/$MDEV $MOUNTPOINT > /dev/null 2>&1 || true
	;;
esac

exit 0

