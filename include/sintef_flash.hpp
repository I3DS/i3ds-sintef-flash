///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#ifndef __I3DS_SINTEF_FLASH_HPP
#define __I3DS_SINTEF_FLASH_HPP

#include <sys/poll.h>

#include <string.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include  <termios.h>
#include <errno.h>

#include <i3ds/flash.hpp>

namespace i3ds
{

    class SintefFlash : public Flash
    {
        public:

            //SintefFlash(std::string device);
            SintefFlash ( NodeID nodeID, std::string port );
            ~SintefFlash();

            // Made public while testing
            void CloseSerialPort();
            void ManualTrigger();
            void setFlashParameters ( int c, float p, float d, float s, float r = 1.0 );
            void SendString ( const char *parameter );
            void SetSerialCommunicationParameters ( int fd );

            void handle_flash ( FlashService::Data &command ) override;

        private:

            int OpenSerialPort ( const char *device );

//  void CloseSerialPort();
// void SendString(const char* parameter);
// void SendParameters(int c, float p, float d, float s, float r);
// void ManualTrigger();



            struct pollfd fds[1];
    };

} // namespace i3ds

#endif
