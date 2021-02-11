#include "KISS_TO_TNC2.h"

bool validateTNC2Frame(const String &tnc2FormattedFrame);

String encode_address_kiss(String tnc2Address, bool isLast);

/*
 * https://ham.zmailer.org/oh2mqk/aprx/PROTOCOLS

	After successfull login, communication carries "TNC2" format
	APRS messages.  Namely text encoding of AX.25 UI frames in
	what became known as "TNC2 monitor style":

	    SOURCE>DESTIN:payload
	    SOURCE>DESTIN,VIA,VIA:payload

	The SOURCE, DESTIN, and VIA fields are AX.25 address fields,
        and have "-SSID" value annexed if the SSID is not zero.
	Also in VIA-fields, if the "HAS BEEN DIGIPEATED" bit is set
	(AX.25 v2 protocol feature) a star ('*') character is appended.
        VIA-fields are separated by comma (',') from DESTIN, and each
        other.

	A double-colon (':') separates address data from payload.
	The payload is passed _AS_IS_ without altering any message
	content bytes, however ending at first CR or LF character
	encountered in the packet.

 */

String encode_kiss(const String& tnc2FormattedFrame) {
    String ax25Frame = "";

    if (validateTNC2Frame(tnc2FormattedFrame)){
        String address = "";
        for (int p=0;p<tnc2FormattedFrame.indexOf(':');p++){
            char currentChar = tnc2FormattedFrame.charAt(p);
            if (currentChar == ':' || currentChar == '>' || currentChar == ','){
                if (currentChar == '>'){
                    // ax25 frame DST SRC
                    // tnc2 frame SRC DST
                    ax25Frame = encode_address_kiss(address, currentChar == ':') + ax25Frame;
                } else {
                    ax25Frame += encode_address_kiss(address, currentChar == ':');
                }
            } else {
                address += currentChar;
            }
        }
        ax25Frame += (char)APRS_CONTROL_FIELD;
        ax25Frame += (char)APRS_INFORMATION_FIELD;
        ax25Frame += tnc2FormattedFrame.substring(tnc2FormattedFrame.indexOf(':')+1);
    }

    String kissFrame = "";
    kissFrame += (char)FEND; // start of frame
    kissFrame += (char)CMD_DATA; // TNC0, DATA
    for (int i = 0; i < ax25Frame.length(); ++i) {
        char currentChar = ax25Frame.charAt(i);
        if (currentChar == (char)FEND) {
            kissFrame += FESC;
            kissFrame += TFEND;
        } else if (currentChar == (char)FESC){
            kissFrame += FESC;
            kissFrame += TFESC;
        } else {
            kissFrame += currentChar;
        }
    }
    kissFrame += (char)FEND; // end of frame
    return kissFrame;
}

/**
 * Encode adress in TNC2 monitor format to ax.25/kiss format
 * @param tnc2Address
 * @param isLast
 * @return
 */
String encode_address_kiss(String tnc2Address, bool isLast) {
    if (tnc2Address.indexOf('-') == -1){
        tnc2Address += "-0";
    }
    bool hasBeenDigipited = tnc2Address.indexOf('*') != -1;

    int ssid = tnc2Address.substring(tnc2Address.indexOf('-')+1).toInt();
    // TODO: SSID should not be > 16
    String kissAddress = "";
    for (int i = 0; i < 6; ++i) {
        if (tnc2Address.length() > i){
            kissAddress += (char)(tnc2Address.charAt(i) << 1);
        } else {
            kissAddress += ' ';
        }
    }
    kissAddress += (char)((ssid << 1) | 0b01100000 | (isLast ? 1 : 0) | (hasBeenDigipited ? 0b10000000 : 0));
    return kissAddress;
}

bool validateTNC2Frame(const String &tnc2FormattedFrame) { return (tnc2FormattedFrame.indexOf(':') != -1) && (tnc2FormattedFrame.indexOf('>') != -1); }
