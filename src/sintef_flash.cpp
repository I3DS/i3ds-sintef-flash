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


i3ds::SintefFlash::SintefFlash (NodeID nodeID, std::string port): Flash(nodeID)
{
  BOOST_LOG_TRIVIAL(info) << "SintefFlash constructor";
  BOOST_LOG_TRIVIAL(info) << "Connecting to serial port: " << port;

  memset (fds, 0, sizeof(pollfd));
  fds[0].fd = OpenSerialPort(port.c_str());

}

i3ds::SintefFlash::~SintefFlash()
{
  BOOST_LOG_TRIVIAL(info) << "i3ds::SintefFlash Destructor";
  CloseSerialPort ();
}

void
i3ds::SintefFlash::CloseSerialPort()
{
  BOOST_LOG_TRIVIAL(info) << "Closing Serial port";
  close(fds[0].fd);
}

void
i3ds::SintefFlash::ManualTrigger ()
{
  BOOST_LOG_TRIVIAL(info) << "Sending manual trigger";
  SendString("TR1");
}

/// This is a parameter string as described in the manual.
void
i3ds::SintefFlash::SendString (const char* parameter)
{

  char buff[100];
  memset (buff, 0, sizeof(buff));
  BOOST_LOG_TRIVIAL(info) << "Sending parameter string: " << parameter;

  char command[100];
  memset (command, 0, sizeof(command));
  strncpy (command, parameter, 99);
  strcat (command, "\n");

  ssize_t retval = write (fds[0].fd, command, strlen (command));
  BOOST_LOG_TRIVIAL(info) << "strlen(command) " << strlen (command) << ",  return value(write): " << retval;
  tcdrain (fds[0].fd);

  // Waiting for response

  fds[0].events = POLLIN;
  fds[0].revents = 0;
  int n;
//  for (;;)
    {
      n = poll (fds, 1, 2000);

      if (n > 0)
	{
	  if (fds[0].revents & POLLIN)
	    { //got data, and look up which fd has data, but we just have one waiting for events
	      ssize_t length;
	      length = read (fds[0].fd, buff, sizeof(buff));
	      if (length == -1)
		{
		  // REMARK: Got "Resource temporary unavailable." sometimes.
		  // But, I think it disapaired when I removed a terminal listening to the same serial port.
		  BOOST_LOG_TRIVIAL(info) <<"Error read event: %s" << strerror (errno);
		}
	      buff[length] = 0;
	     // BOOST_LOG_TRIVIAL(info) << "From poll \"" << buff<<"\"";
	      // TODO: Potencial bufferflow ?
	      char buff2[100];
	      int j = 0;
	      for(int i=0; buff[i] != '\0'; i++)
		{
		  if(buff[i] == '\n')
		    {
		      strncpy(&buff2[j], "<NL>", 4);
		      j = j + 3;
		    }
		  else
		    {
		      buff2[j] = buff[i];
		    }
		  buff2[j+1] = '\0';
		  j++;
		}

	      BOOST_LOG_TRIVIAL(info) << "From poll:(Newline replaced) \"" << buff2<<"\"";
	      BOOST_LOG_TRIVIAL(info) << "Received Length:" << length;
	      if (buff[strlen (command) + 1] == '>')
		{
		  BOOST_LOG_TRIVIAL(info) << "Ok response.\n";
		 if(strstr(buff, "Err 5") != NULL)
		     {
		       BOOST_LOG_TRIVIAL(info) << "Outside limits for combination strength & duration.";
		       throw i3ds::CommandError(error_value, "Outside limits for combination strength & duration.");
		     }

		   if(strstr(buff, "Err ") != NULL)
		     {
		       BOOST_LOG_TRIVIAL(info) << "Error in data string sent to serial port.";
		       throw i3ds::CommandError(error_value, "Error in data string sent to serial port.");
		     }





		}
	      else
		{
		  // Reformating error code to remove OK prompt at new line before sending it to client.
		  BOOST_LOG_TRIVIAL(info) << "Error in response: %s" << buff;
		  char *p;
		  p = strchr (buff, '\n');
		  *p = '\0';
		  BOOST_LOG_TRIVIAL(info) << "Error in response: %s" << buff;
		}
	    }
	  else
	    {
	      BOOST_LOG_TRIVIAL(info) <<"Other event type? Needs to be handed?";
	    }
	}
      else
	{
	  BOOST_LOG_TRIVIAL(info) << "No data within 2 seconds.";
	}
    }

/*

     if(strstr(buff, "Err 5" != NULL)
	 {
	   BOOST_LOG_TRIVIAL(info) << "Outside limits for combination strength & duration.";
	   throw i3ds::CommandError(error_value, "Outside limits for combination strength & duration. Strength used. Duration adjusted.");
	 }

     if(strstr(buff, "Err " != NULL)
	 {
	   BOOST_LOG_TRIVIAL(info) << "Error in data string sent to serial port.";
	   throw i3ds::CommandError(error_value, "Error in data string sent to serial port.");
          }



*/

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

  BOOST_LOG_TRIVIAL(info) <<"Sending configuration parameters: " << "c:" << c << " "
      "p:" << p << " "
      "d:" << d << " "
      "s:" << s << " ";

  if (r != -1.0)
    {
      BOOST_LOG_TRIVIAL(info) << "r: " << r;
    }


  if (r != -1.0)
    {

      sprintf (buffer, "RT%d,%g,%g,%g,%g", c, p, d, s, r);
    }
  else
    {
      sprintf (buffer, "RT%d,%g,%g,%g", c, p, d, s);
    }

  SendString (buffer);

}

void
i3ds::SintefFlash::SetSerialCommunicationParameters (int fd)
{
  BOOST_LOG_TRIVIAL(info) << "setConfiguration";
  struct termios Opt;
  tcgetattr (fd, &Opt);
  cfsetispeed (&Opt, B115200);
  cfsetospeed (&Opt, B115200);
  tcsetattr (fd, TCSANOW, &Opt);

  struct termios options;
  tcgetattr (fd, &options);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  int retcode = tcsetattr (fd, TCSANOW, &options);
  BOOST_LOG_TRIVIAL(info) << "tcsetattr return code: " << retcode;
  return;
}

int
i3ds::SintefFlash::OpenSerialPort(const char *device)
{
  int fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY);  //| O_NOCTTY | O_NDELAY
  if (-1 == fd)
    {
      BOOST_LOG_TRIVIAL(info) << "Can't Open Serial Port";
      return -1;
    }
  else
    {
      SetSerialCommunicationParameters (fd);
      return fd;
    }

}

void
i3ds::SintefFlash::handle_flash(FlashService::Data& command)
{
  BOOST_LOG_TRIVIAL(info) << "handle_flash";
  int strength = command.request.strength;
  int duration = command.request.duration;

  BOOST_LOG_TRIVIAL(info) << "Strength: " << strength;
  BOOST_LOG_TRIVIAL(info) << "Duration: " << duration;

  if ((strength < 0) || (strength > 100) )
    {
      BOOST_LOG_TRIVIAL(info) << "Strength must be within 0-100%.";
      throw i3ds::CommandError(error_value, "Flash Strength must be within 0-100%.");
    }

  if ((duration < 0) || (duration > 3000) )
    {
      BOOST_LOG_TRIVIAL(info) << "Duration must be within 0-3000ms.";
      throw i3ds::CommandError(error_value, "Flash duration must be within 0-3000ms.");
    }


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



    setFlashParameters (
			  1, 			// Configure strobe output
			  duration,		// Pulse width ms
			  0.01, 		// Delay from trigger to pulse in ms(0.01 to 999)
			  strength		/// Settings in percent
    ); 			// 5th parameter retrigger delay in ms(optional not used)


// Remember that the trigger related stuff is controlled by the camera its self.
}
