with Ada.Command_Line;
with Ada.Real_Time;
with Ada.Text_IO;
with Beacon;
with Beacon.Bridge;
with Beacon.Transport.UART;
with Beacon.Wire;
with Telemetry;
with Telemetry.Codecs;

procedure UART_CRC_Test is
   use Ada.Command_Line;
   use Ada.Real_Time;
   use Ada.Text_IO;
   use type Beacon.Wire.Byte;
   use type Telemetry.Sensor_Id;

   package UART renames Beacon.Transport.UART;
   package Wire renames Beacon.Wire;
   package Topic renames Telemetry.Temperature_Topic;

   package Test_Bridge is new Beacon.Bridge
     (Local       => Topic,
      Topic_Id    => Telemetry.Temperature_Topic_Id,
      Serialize   => Telemetry.Codecs.Serialize_Reading,
      Deserialize => Telemetry.Codecs.Deserialize_Reading);

   Tx_Name : constant String :=
     (if Argument_Count >= 1 then Argument (1) else "COM20");

   Rx_Name : constant String :=
     (if Argument_Count >= 2 then Argument (2) else "COM21");

   Tx  : UART.UART_Transport;
   Rx  : aliased UART.UART_Transport;
   Sub : Topic.Subscriber (Beacon.Best_Effort);

   Payload : Wire.Byte_Array (1 .. Wire.Max_Payload);
   Frame   : Wire.Byte_Array (1 .. Wire.Max_Frame);
   PL      : Natural;
   FL      : Natural;

   Received : Telemetry.Reading;
   Got      : Boolean;
   Seen     : Natural := 0;
   Deadline : Time;

begin
   Put_Line ("Beacon UART bridge CRC test");

   Topic.Create_Subscriber (Sub);

   UART.Open (Rx, UART.Serial.Port_Name (Rx_Name));
   UART.Open (Tx, UART.Serial.Port_Name (Tx_Name));

   Test_Bridge.Start_Ingress (Rx'Access);

   Telemetry.Codecs.Serialize_Reading
     ((Sensor => 7, Sequence => 42, Temperature => 4.2),
      Payload, PL);

   Wire.Encode
     (Topic    => Telemetry.Temperature_Topic_Id,
      Sequence => 0,
      Payload  => Payload (1 .. PL),
      Frame    => Frame,
      Last     => FL);

   --  Corrupt one payload byte after CRC calculation.
   Frame (Wire.Header_Length + 1) :=
     Frame (Wire.Header_Length + 1) xor 1;

   UART.Send (Tx, Frame (1 .. FL));

   --  Restore the payload and send the valid frame.
   Frame (Wire.Header_Length + 1) :=
     Frame (Wire.Header_Length + 1) xor 1;

   UART.Send (Tx, Frame (1 .. FL));

   Deadline := Clock + Seconds (5);

   loop
      Topic.Try_Receive (Sub, Received, Got);

      if Got then
         Seen := Seen + 1;

         if Received.Sequence /= 42
           or else Received.Sensor /= 7
         then
            raise Program_Error with
              "Unexpected UART subscriber message";
         end if;
      else
         delay 0.001;
      end if;

      exit when Clock >= Deadline
        or else
          (Seen >= 1 and then Test_Bridge.Rejected_Frames >= 1);
   end loop;

   if Seen /= 1 then
      raise Program_Error with
        "Expected exactly one delivered message, got"
        & Seen'Image;
   end if;

   if Test_Bridge.Rejected_Frames /= 1 then
      raise Program_Error with
        "Expected exactly one CRC rejection, got"
        & Test_Bridge.Rejected_Frames'Image;
   end if;

   Put_Line ("  ok    corrupted frame rejected");
   Put_Line ("  ok    valid frame delivered");
   Put_Line ("  ok    subscriber received exactly one message");

   Test_Bridge.Stop;
   delay 0.2;
   UART.Close (Tx);
   UART.Close (Rx);

   Put_Line ("all passed");
end UART_CRC_Test;
