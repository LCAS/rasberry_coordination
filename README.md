# start server
    needs installation of websock: sudo pip install webnsock
    run as WEBSOCKET_URL="wss://lcas.lincoln.ac.uk/car/ws" python callarobot.py

# start client

    needs installation of https://github.com/websocket-client/websocket-client: sudo pip install websocket-client
    run as WEBSOCKET_URL="wss://lcas.lincoln.ac.uk/car/ws" python ws_client.py

