with Ada.Text_IO;  use Ada.Text_IO;
with Beacon.Transport.UART;
with Beacon.Wire;  use Beacon.Wire;
with Ada.Command_Line;
with Ada.Streams;
with GNAT.Serial_Communications;

procedure UART_Tests is

   package UART renames Beacon.Transport.UART;
   package Serial renames GNAT.Serial_Communications;

   Tx : UART.UART_Transport;
   Rx : UART.UART_Transport;

   Payload : Byte_Array (1 .. Max_Payload);
   Sent    : Byte_Array (1 .. Max_Frame);
   Result  : Byte_Array (1 .. Max_Frame);
   Decoded : Byte_Array (1 .. Max_Payload);

   Last         : Natural;
   Count        : Natural;
   Payload_Last : Natural;
   Topic        : Topic_Id_Type;
   Sequence     : Sequence_Type;
   Status       : Decode_Status;

   Tx_Port : constant String :=
     (if Ada.Command_Line.Argument_Count >= 1
      then Ada.Command_Line.Argument (1)
      else "COM20");

   Rx_Port : constant String :=
     (if Ada.Command_Line.Argument_Count >= 2
      then Ada.Command_Line.Argument (2)
      else "COM21");

   use Ada.Streams;

   Raw_Port : Serial.Serial_Port;

   --  COBS encoding of fourteen zero bytes:
   --  fifteen 0x01 code bytes followed by a 0x00 delimiter.
   First_Half : constant Stream_Element_Array (1 .. 7) :=
     [others => 1];

   Second_Half : constant Stream_Element_Array (1 .. 9) :=
     [1 => 1, 2 => 1, 3 => 1, 4 => 1,
      5 => 1, 6 => 1, 7 => 1, 8 => 1,
      9 => 0];

   --  Invalid COBS packet: code 5 requires four data bytes,
   --  but the packet ends immediately.
   Malformed : constant Stream_Element_Array (1 .. 2) :=
     [5, 0];

   --  Deliberately exceeds the UART receive buffer.
   Oversized : constant Stream_Element_Array (1 .. 1_200) :=
     [1 .. 1_199 => 1, 1_200 => 0];

begin
   Put_Line ("Beacon UART maximum-frame test");

   --  Include zero bytes and nonzero values throughout the payload.
   for I in Payload'Range loop
      Payload (I) := Byte (I mod 256);
   end loop;

   Encode
     (Topic    => 7,
      Sequence => 42,
      Payload  => Payload,
      Frame    => Sent,
      Last     => Last);

   UART.Open (Rx, UART.Serial.Port_Name (Rx_Port));
   UART.Open (Tx, UART.Serial.Port_Name (Tx_Port));

   --  No data available: Receive must return without a frame.
   UART.Receive (Rx, Result, Count, 0.1);

   if Count /= 0 then
      raise Program_Error with
        "Expected zero bytes on receive timeout";
   end if;

   Put_Line ("  ok    empty receive timeout");

   UART.Send (Tx, Sent (1 .. Last));
   UART.Receive (Rx, Result, Count, 5.0);

   if Count /= Max_Frame then
      raise Program_Error with
        "Maximum frame length incorrect:" & Count'Image;
   end if;

   if Result (1 .. Count) /= Sent (1 .. Last) then
      raise Program_Error with "UART frame differs from transmitted data";
   end if;

   Decode
     (Result (1 .. Count), Topic, Sequence,
      Decoded, Payload_Last, Status);

   if Status /= Ok
     or else Topic /= 7
     or else Sequence /= 42
     or else Payload_Last /= Max_Payload
     or else Decoded /= Payload
   then
      raise Program_Error with "UART frame validation failed";
   end if;

   --  Send another complete frame without reopening either port.
   UART.Send (Tx, Sent (1 .. Last));
   UART.Receive (Rx, Result, Count, 5.0);

   if Count /= Last then
      raise Program_Error with
        "Second UART frame length incorrect:" & Count'Image;
   end if;

   if Result (1 .. Count) /= Sent (1 .. Last) then
      raise Program_Error with
        "Second UART frame differs from transmitted data";
   end if;

   Put_Line ("  ok    consecutive frame reception");

   --  Stop using the normal transmitting transport.
   UART.Close (Tx);

   Serial.Open
  (Raw_Port,
   Serial.Port_Name
     ((if Tx_Port'Length >= 3
         and then Tx_Port (Tx_Port'First .. Tx_Port'First + 2) = "COM"
       then "\\.\" & Tx_Port
       else Tx_Port)));

   Serial.Set
     (Port      => Raw_Port,
      Rate      => Serial.B115200,
      Bits      => Serial.CS8,
      Parity    => Serial.None,
      Stop_Bits => Serial.One,
      Block     => False,
      Timeout   => 0.01);

   --  Send the first portion without the delimiter.
   Serial.Write (Raw_Port, First_Half);

   --  The receiver must time out without returning a partial frame.
   UART.Receive (Rx, Result, Count, 0.2);

   if Count /= 0 then
      raise Program_Error with "Partial UART frame returned prematurely";
   end if;

   Put_Line ("  ok    partial frame timeout");

   --  Complete the same COBS packet.
   Serial.Write (Raw_Port, Second_Half);

   UART.Receive (Rx, Result, Count, 5.0);

   if Count /= Header_Length then
      raise Program_Error with
        "Recovered frame length incorrect:" & Count'Image;
   end if;

   for I in 1 .. Count loop
      if Result (I) /= 0 then
         raise Program_Error with "Recovered frame data mismatch";
      end if;
   end loop;

   Put_Line ("  ok    partial frame recovery");

   --  Send an invalid COBS packet.
   Serial.Write (Raw_Port, Malformed);

   UART.Receive (Rx, Result, Count, 0.2);

   if Count /= 0 then
      raise Program_Error with
        "Malformed COBS packet was incorrectly accepted";
   end if;

   Put_Line ("  ok    malformed frame rejected");

   --  Follow with a valid COBS packet without reopening the port.
   Serial.Write (Raw_Port, First_Half);
   Serial.Write (Raw_Port, Second_Half);

   UART.Receive (Rx, Result, Count, 5.0);

   if Count /= Header_Length then
      raise Program_Error with
        "UART failed to recover after malformed COBS packet";
   end if;

   for I in 1 .. Count loop
      if Result (I) /= 0 then
         raise Program_Error with
           "Recovered COBS packet data mismatch";
      end if;
   end loop;

   Put_Line ("  ok    malformed frame recovery");

   --  Transmit an oversized COBS packet.
   Serial.Write (Raw_Port, Oversized);

   UART.Receive (Rx, Result, Count, 5.0);

   if Count /= 0 then
      raise Program_Error with
        "Oversized UART packet was incorrectly accepted";
   end if;

   Put_Line ("  ok    oversized frame rejected");

   --  Verify recovery after the oversized packet.
   Serial.Write (Raw_Port, First_Half);
   Serial.Write (Raw_Port, Second_Half);

   UART.Receive (Rx, Result, Count, 5.0);

   if Count /= Header_Length then
      raise Program_Error with
        "UART failed to recover after oversized packet";
   end if;

   for I in 1 .. Count loop
      if Result (I) /= 0 then
         raise Program_Error with
           "Recovered frame data mismatch after oversized packet";
      end if;
   end loop;

   Put_Line ("  ok    oversized frame recovery");

   Serial.Close (Raw_Port);

   --  Verify failure when opening a nonexistent serial port.
   declare
      Test_Port : UART.UART_Transport;
      Failed    : Boolean := False;

      Invalid_Port : constant String :=
      (if Tx_Port'Length >= 3
            and then Tx_Port (Tx_Port'First .. Tx_Port'First + 2) = "COM"
         then "COM250"
         else "/tmp/beacon-nonexistent-uart-port");
   begin
      begin
         UART.Open
         (Test_Port, UART.Serial.Port_Name (Invalid_Port));
      exception
         when Serial.Serial_Error =>
            Failed := True;
      end;

      if not Failed then
         UART.Close (Test_Port);
         raise Program_Error with
           "Nonexistent UART port was accepted";
      end if;

      Put_Line ("  ok    invalid serial port rejected");

      --  The same transport object must remain reusable.
      UART.Open (Test_Port, UART.Serial.Port_Name (Tx_Port));
      UART.Close (Test_Port);

      Put_Line ("  ok    recovery after failed port open");
   end;

   UART.Close (Tx);
   UART.Close (Rx);

   Put_Line ("  ok    maximum 1038-byte frame");
   Put_Line ("  ok    COBS round trip");
   Put_Line ("  ok    Beacon.Wire CRC and payload");
   Put_Line ("all passed");

exception
   when others =>
      UART.Close (Tx);
      UART.Close (Rx);
      raise;
end UART_Tests;
