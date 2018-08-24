///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include "sintef_flash.hpp"

#include <boost/program_options.hpp>

#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace po = boost::program_options;
namespace logging = boost::log;


volatile bool running;

// startup parameters
// set /dev
// Send manual trigger
// Send Configuration string
// send configuration parameters
// Report result?
// Standard parameters for rest of communication system.

int
main (int argc, char* argv[])
{
  unsigned int node_id;
  std::string serialPort;
  std::string flashCommand;
  bool triggerFlash = false;
  bool dontRunProgram = false;
  std::vector<double> commandParameters;
  std::vector<double> remoteParameters;

  po::options_description desc ("Wide angle flash control options");
  desc.add_options () ("help,h", "Produce this message")
  ("node,n", po::value<unsigned int>(&node_id)->required(), "Node ID of camera")

  ("device,d", po::value<std::string> ()->default_value (DEFAULT_SERIAL_PORT),
   "Serial(USB) port the \"Wide angle flash\" is connected to")

  ("trigger-flash,t", "Trigger flash manually")

  ("command,c", po::value<std::string> (),
   "Manually send command string to interact with the flash.\n"
   "\tOne may use all command string described in the manual.\n"
   "\tE.g \"TR1\" for triggering flash.\n") (
      "remote,r",
      po::value < std::vector<double> > (&commandParameters)->multitoken (),
      "This is the command available remotely via i3ds."
      "Actually the used command is: \"RTc,p,d,s,r\"\n"
      "\tc=1 => Light strobe output.\n\tc=2 => Trigger output signal\n"
      "\tp= pulse width in milliseconds (0.01 to 3)\n"
      "\ts= setting in percent(0 to 100)\n"
      "\tr= re-trigger delay in milliseconds (an optional parameter)\n"
      "Settings are not keep during a power cycle.\n"
      "Use format: -c 1 0.1 200 100 12\n")

  //("vector_value", po::value<std::vector<double> >(&vecoption)->
  //     multitoken()->default_value(std::vector<double>{0, 1, 2}), "description");

  ("verbose,v", "Print verbose output") ("quite,q", "Quiet output");

  po::variables_map vm;
  po::store (po::parse_command_line (argc, argv, desc), vm);

  if (vm.count ("help"))
    {
      BOOST_LOG_TRIVIAL(info) << desc;
      return -1;
    }

  if (vm.count ("device"))
    {
      serialPort = vm["device"].as<std::string> ();
      BOOST_LOG_TRIVIAL(info) << "Using serial port: " << serialPort;
    }
  if (vm.count ("trigger-flash"))
    {
      // serialPort = vm["device"].as<std::string>();
      BOOST_LOG_TRIVIAL(info)<< "Manually trigger flash";
      triggerFlash = true;
      dontRunProgram = true;
    }

  if (vm.count ("command"))
    {
      flashCommand = vm["command"].as<std::string> ();
      BOOST_LOG_TRIVIAL(info) << "Manually send command to flash.";
      BOOST_LOG_TRIVIAL(info) << "Command:" << flashCommand;
      dontRunProgram = true;
    }

  if (vm.count ("remote"))
    {
      remoteParameters = vm["remote"].as<std::vector<double>> ();
      BOOST_LOG_TRIVIAL(info) << "Test remote parameters.";
      BOOST_LOG_TRIVIAL(info) <<  "Commandparameter[1]:" << flashCommand << remoteParameters[0];
      dontRunProgram = true;
    }

  if (vm.count ("quite"))
    {
      logging::core::get ()->set_filter (
	  logging::trivial::severity >= logging::trivial::warning);
    }
  else if (!vm.count ("verbose"))
    {
      logging::core::get ()->set_filter (
	  logging::trivial::severity >= logging::trivial::info);
    }

  po::notify (vm);



  BOOST_LOG_TRIVIAL(info) << "Wide angle flash got node ID: " << node_id;

  BOOST_LOG_TRIVIAL(trace) << "---> [OK]";

  i3ds::Context::Ptr context(i3ds::Context::Create());
  i3ds::Server server(context);

  i3ds::SintefFlash serialCommunicator(node_id, serialPort);

  BOOST_LOG_TRIVIAL(info) << "Attaching server.";
  serialCommunicator.Attach(server);
  BOOST_LOG_TRIVIAL(info) << "Starting server.";
  server.Start();
  BOOST_LOG_TRIVIAL(info) << "Server started.";



  if (!flashCommand.empty ())
    {
      serialCommunicator.SendString (flashCommand.c_str ());
    }

  if (!remoteParameters.empty ())
    {
      if (remoteParameters.size () == 4)
	{
      serialCommunicator.setCommunicationParameters (
	  remoteParameters[0],
	  remoteParameters[1],
	  remoteParameters[2],
	  remoteParameters[3]
      );
    }
  if (remoteParameters.size () == 5)
    {
  serialCommunicator.setCommunicationParameters (
      remoteParameters[0],
      remoteParameters[1],
      remoteParameters[2],
      remoteParameters[3],
      remoteParameters[4]
  );
}
}

if (triggerFlash)
{
serialCommunicator.ManualTrigger ();
}

if (dontRunProgram)
{
return 0;
}
else
  {
    running = true;

    while (running)
        {
          sleep(1);
        }

      server.Stop();
  }


/*
 serialCommunicator->sendParameterString ("RT1,3,200,10.0");

 serialCommunicator->sendManualTrigger ();
 serialCommunicator->sendParameterString ("RT1d3,300.2,1d");
 sleep(1);
 serialCommunicator->sendManualTrigger ();
 sleep(1);

 serialCommunicator->setFlashParameters (1, 0.2, 0.01, 90);
 sleep(1);
 serialCommunicator->sendManualTrigger ();
 */
serialCommunicator.CloseSerialPort ();

return 0;
}
