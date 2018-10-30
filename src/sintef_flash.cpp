///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "sintef_flash.hpp"



#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace logging = boost::log;


i3ds::SintefFlash::SintefFlash ( NodeID nodeID, std::string port ) : Flash ( nodeID )
{
    BOOST_LOG_TRIVIAL ( info ) << "SintefFlash constructor";
    BOOST_LOG_TRIVIAL ( info ) << "Connecting to serial port: " << port;

    memset ( fds, 0, sizeof ( pollfd ) );
    fds[0].fd = OpenSerialPort ( port.c_str() );

}

i3ds::SintefFlash::~SintefFlash()
{
    BOOST_LOG_TRIVIAL ( info ) << "i3ds::SintefFlash Destructor";
    CloseSerialPort ();
}

void
i3ds::SintefFlash::CloseSerialPort()
{
    BOOST_LOG_TRIVIAL ( info ) << "Closing Serial port";
    close ( fds[0].fd );
}

void
i3ds::SintefFlash::ManualTrigger ()
{
    BOOST_LOG_TRIVIAL ( trace ) << "Sending manual trigger";
    SendString ( "TR1" );
}

/// This is a parameter string as described in the manual.
void
i3ds::SintefFlash::SendString ( const char *parameter )
{
    const int lengthOfBuff = 100;
    char buff[lengthOfBuff];
    memset ( buff, 0, sizeof ( buff ) );
    BOOST_LOG_TRIVIAL ( trace ) << "Sending parameter string: " << parameter;

    char command[lengthOfBuff];
    memset ( command, 0, lengthOfBuff - 1 );
    strncpy ( command, parameter, lengthOfBuff - 1 );
    strcat ( command, "\n" );

    tcflush ( fds[0].fd,  TCIFLUSH );


    ssize_t retval = write ( fds[0].fd, command, strlen ( command ) );
    BOOST_LOG_TRIVIAL ( trace ) << "strlen(command) " << strlen ( command ) << ",  return value(write): " << retval;
    tcdrain ( fds[0].fd );


    // Waiting for response

    fds[0].events = POLLIN;
    fds[0].revents = 0;
    int n;

    n = poll ( fds, 1, 2000 );

    if ( n > 0 )
    {
        if ( fds[0].revents & POLLIN )
        {
            //got data, and look up which fd has data, but we just have one waiting for events
            ssize_t length;
            length = read ( fds[0].fd, buff, lengthOfBuff );
            if ( length == -1 )
            {
                // REMARK: Got "Resource temporary unavailable." sometimes.
                // But, I think it disapaired when I removed a terminal listening to the same serial port.
                BOOST_LOG_TRIVIAL ( trace ) << "Error read event: " << strerror ( errno );
            }
            buff[length] = 0;
            // BOOST_LOG_TRIVIAL(info) << "From poll \"" << buff<<"\"";

            // This part replaces '\n' with '<NL>' for debug printout purposes
            char buff2[2 * lengthOfBuff];
            int j = 0;
            for ( int i = 0; ( buff[i] != '\0' ) && ( i < lengthOfBuff ); i++ )
            {
                if ( buff[i] == '\n' )
                {
                    strncpy ( &buff2[j], "<NL>", 4 );
                    j = j + 3;
                }
                else
                    if ( buff[i] == '\r' )
                    {

                        strncpy ( &buff2[j], "<CR>", 4 );
                        j = j + 3;
                    }
                    else
                    {
                        buff2[j] = buff[i];
                    }
                buff2[j + 1] = '\0';
                j++;
            }

            BOOST_LOG_TRIVIAL ( trace ) << "From poll:(NL & CR replaced) \"" << buff2 << "\"";
            BOOST_LOG_TRIVIAL ( trace ) << "Received Length:" << length;


            BOOST_LOG_TRIVIAL ( trace ) << "Ok response.\n";
            if ( strstr ( buff, "Err 5" ) != NULL )
            {
                BOOST_LOG_TRIVIAL ( trace ) << "Outside limits for combination strength & duration. Strength kept, duration adjusted";
                throw i3ds::CommandError ( error_value, "Outside limits for combination strength & duration. Strength kept, duration adjusted" );
            }

            if ( strstr ( buff, "Err " ) != NULL )
            {
                BOOST_LOG_TRIVIAL ( warning ) << "Error in data string sent to serial port.";
                throw i3ds::CommandError ( error_value, "Error in data string sent to serial port: " + std::string ( buff ) );
            }
        }
        else
        {
            BOOST_LOG_TRIVIAL ( warning ) << "Other event type? Needs to be handled?";
        }
    }
    else
    {
        BOOST_LOG_TRIVIAL ( warning ) << "No data received from serial port within 2 seconds.";
        throw i3ds::CommandError ( error_other, "No data received from serial port within 2 seconds." );

    }


}

//RTc,p,d,s,
//RTc,p,d,s,r (r is optional)
// Limits are described in manual#include <boost/program_options.hpp>
void
i3ds::SintefFlash::setFlashParameters ( int c, 	///< 1 – light strobe output; 2 – Trigger output signal.
                                        float p, ///< Pulsewidth in milliseconds (0.01 to 3)
                                        float d, ///< Delay from trigger to pulse in milliseconds (0.01 to 999)
                                        float s, ///< Strength setting i percent (0 to 100)
                                        float r 	///< Re-trigger delay in milliseconds (optional parameter)
                                      )
{

    char buffer[100];

    BOOST_LOG_TRIVIAL ( trace ) << "Sending configuration parameters: " << "c:" << c << " "
                                "p:" << p << " "
                                "d:" << d << " "
                                "s:" << s << " ";

    if ( r != -1.0 )
    {
        BOOST_LOG_TRIVIAL ( trace ) << "r: " << r;
        sprintf ( buffer, "RT%d,%g,%g,%g,%g", c, p, d, s, r );
    }
    else
    {
        sprintf ( buffer, "RT%d,%g,%g,%g", c, p, d, s );
    }

    SendString ( buffer );

}

void
i3ds::SintefFlash::SetSerialCommunicationParameters ( int fd )
{
    BOOST_LOG_TRIVIAL ( info ) << "setConfiguration";
    struct termios Opt;
    tcgetattr ( fd, &Opt );
    cfsetispeed ( &Opt, B115200 );
    cfsetospeed ( &Opt, B115200 );
    tcsetattr ( fd, TCSANOW, &Opt );

    struct termios options;
    tcgetattr ( fd, &options );
    options.c_lflag &= ~ ( ICANON | ECHO | ECHOE | ISIG );
    options.c_oflag &= ~OPOST;
    int retcode = tcsetattr ( fd, TCSANOW, &options );
    BOOST_LOG_TRIVIAL ( info ) << "tcsetattr return code: " << retcode;
    return;
}

int
i3ds::SintefFlash::OpenSerialPort ( const char *device )
{
    int fd = open ( device, O_RDWR | O_NOCTTY | O_NDELAY ); //| O_NOCTTY | O_NDELAY
    if ( -1 == fd )
    {
        BOOST_LOG_TRIVIAL ( warning ) << "Can't Open Serial Port";
        BOOST_LOG_TRIVIAL ( warning ) << "Exiting.";
        exit ( 1 );
        return -1;
    }
    else
    {
        SetSerialCommunicationParameters ( fd );
        return fd;
    }

}



/// REMARK: We are not handling spontainous overhead messages
/// REMARK: Duty cycle limitations has to be handled by camera which knows the parameters.
void
i3ds::SintefFlash::handle_flash ( FlashService::Data &command )
{
    BOOST_LOG_TRIVIAL ( info ) << "handle_flash";
    int flash_strength = command.request.strength;
    int duration_us = command.request.duration;

    BOOST_LOG_TRIVIAL ( info ) << "Strength: " << flash_strength;
    BOOST_LOG_TRIVIAL ( info ) << "Duration in us: " << duration_us;

    if ( ( flash_strength < 0 ) || ( flash_strength > 100 ) )
    {
        BOOST_LOG_TRIVIAL ( warning ) << "Strength must be within 0-100%.";
        throw i3ds::CommandError ( error_value, "Flash strength must be within 0-100%." );
    }

    if ( ( duration_us < 10 ) || ( duration_us > 3000000 ) )
    {
        BOOST_LOG_TRIVIAL ( warning ) << "Duration must be within 0.01-3ms.";
        throw i3ds::CommandError ( error_value, "Flash duration must be within 0.01-3ms." );
    }
    float flash_duration_ms = duration_us / 1000.;

    /// Remark: Err 5 is a out of range warning for one parameter.
    /// It is fixed to valid value.
    /// But, it looks as there is a limit with a relationship between duration and flash strength
    /// Explained here:
    //   http://www.gardasoft.com/downloads/?f=112
    //  Page 13(Read it)
    ///
    //Output brightness		850nm variant    | 			|	White variant
    //			Allowed pulsewidth |Allowed duty cycle 	| Allowed pulse width   |   Allowed duty cycle
    // 0% to  20% 		3ms 			6% 			3ms 			3%
    //21% to  30% 		3ms 			6%	 		2ms 			3%
    //31% to  50% 		3ms 			3% 			2ms 			2%
    //51% to 100% 		2ms 			3% 			1ms 			1%



    // After some, expremental testing.(It is not described in manual.)
    // It looks like there is problem with the strength-duration ratio.
    // It vil give the Err 5 warning. Set the actual strength and decrease duration of the pulse to fit the constraints



    // Test limitation of duration vs strength.
    if ( flash_strength > 51 )
      {
	if ( flash_duration_ms > 1. )
	  {
	    flash_duration_ms = 1.;
	  }
      }

    if ( ( flash_strength >= 21 ) && ( flash_strength <= 50 ) )
      {
	if ( flash_duration_ms > 2. )
	  {
	    flash_duration_ms = 2.;
	  }
      }

    if ( flash_strength <= 20 )
      {
	if ( flash_duration_ms > 3. )
	  {
	    flash_duration_ms = 3.;
	  }
      }


    setFlashParameters (
        1, 			// Configure strobe output
        flash_duration_ms,		// Pulse width ms
        0.01, 		// Delay from trigger to pulse in ms(0.01 to 999)
        flash_strength		/// Settings in percent
    ); 			// 5th parameter retrigger delay in ms(optional not used)


// Remember that the trigger related stuff is controlled by the camera its self.
}
