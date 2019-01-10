typedef struct header {
	char ID_String[6];
	int sample_counter;
	char datagram_counter;
	char number_of_items;
	int time_code;
	char character_ID;
	char num_of_bodyseg;
	char num_of_props;
	char num_of_traking_data_seg;
	char reserved_for_future_use[2];
	char size_of_payload[2];
}Header;