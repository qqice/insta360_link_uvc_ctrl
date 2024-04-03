#!/bin/sh
cp poco_lib/libPocoCrypto.so /usr/local/lib/libPocoCrypto.so.64
cp poco_lib/libPocoFoundation.so /usr/local/lib/libPocoFoundation.so.64
cp poco_lib/libPocoJSON.so /usr/local/lib/libPocoJSON.so.64
cp poco_lib/libPocoNet.so /usr/local/lib/libPocoNet.so.64
cp poco_lib/libPocoNetSSL.so /usr/local/lib/libPocoNetSSL.so.64
cp poco_lib/libPocoUtil.so /usr/local/lib/libPocoUtil.so.64
cp poco_lib/libPocoXML.so /usr/local/lib/libPocoXML.so.64
cd /usr/local/lib/
ln -s libPocoCrypto.so.64 libPocoCrypto.so
ln -s libPocoFoundation.so.64 libPocoFoundation.so
ln -s libPocoJSON.so.64 libPocoJSON.so
ln -s libPocoNet.so.64 libPocoNet.so
ln -s libPocoNetSSL.so.64 libPocoNetSSL.so
ln -s libPocoUtil.so.64 libPocoUtil.so
ln -s libPocoXML.so.64 libPocoXML.so