typedef struct header {
	char ID_String[6];
	int sample_counter;
	char datagram_counter;
	char number_of_items;
	int time_code;
	char character_ID;
	char reserved_for_future_use[7];
}Header;