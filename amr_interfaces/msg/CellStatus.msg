# Estado de la celda (PLC) visto por amr_cell/cell_client.
std_msgs/Header header
string cell_id
bool broker_connected
bool plc_online            # false: sin plc/status o LWT "offline"
string state               # ready | busy | fault | offline | unknown
string detail
