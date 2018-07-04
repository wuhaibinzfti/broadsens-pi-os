build wpa_supplicant tool:
1. build libnl
  1) cd libnl;tar -xvf libnl-3.4.0.tar.gz;cd ..
  2) make libnl-config
  3) make libnl
  4) make libnl-install
  
2. build openssl
  1) cd openssl;tar -xvf openssl-1.0.2o.tar.gz;cd ..
  2) make openssl-config
  3) vi openssl/openssl-1.0.2o/Makefile [search '-m64' options & delet]
     modify info:
         CFLAG= -fPIC -DOPENSSL_PIC -DOPENSSL_THREADS -D_REENTRANT -DDSO_DLFCN -DHAVE_DLFCN_H -march=armv7-a -m64 -DL_ENDIAN -O3 -Wall
		 -->CFLAG= -fPIC -DOPENSSL_PIC -DOPENSSL_THREADS -D_REENTRANT -DDSO_DLFCN -DHAVE_DLFCN_H -march=armv7-a -DL_ENDIAN -O3 -Wall
		 SHARED_LDFLAGS=-m64
		 -->SHARED_LDFLAGS=
		 
	 quit & save
  4) make openssl
  5) make openssl-install
  
 3. build wpa_supplicant tool
  1) cd wpa_supplicant;tar -xvf wpa_supplicant-2.6.tar.gz;cd ..
  2) make wpa_supplicant-config
  3) export PKG_CONFIG_PATH=$PWD/output/libnl/lib/pkgconfig/:$PKG_CONFIG_PATH
  4) make wpa_supplicant
  
 4 add to target list:
   1) libnl
       libnl-3.so 
       libnl-3.so.200  
       libnl-3.so.200.26.0
   
       libnl-genl-3.so 
       libnl-genl-3.so.200   
       libnl-genl-3.so.200.26.0
	   
	   cp build/libnl/lib/libnl-3.so* [rootfs/lib path]
	   cp build/libnl/lib/libnl-genl-3.so* [rootfs/lib path]
   2) openssl
       libssl.so
	   libssl.so.1.0.0
	   
       libcrypto.so
	   libcrypto.so.1.0.0
	   
	   cp build/openssl/lib/*.so* [rootfs/lib path]
   
   3)  wpa_supplicant tool
       wpa_cli
       wpa_passphrase
	   wpa_supplicant

