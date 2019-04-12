#!/bin/bash
g++ tls-alex-client.cpp make_tls_client.cpp tls_client_lib.cpp tls_pthread.cpp tls_common_lib.cpp -pthread -lssl -lncurses -lcrypto -o tls-alex-client
echo "compiled" 
