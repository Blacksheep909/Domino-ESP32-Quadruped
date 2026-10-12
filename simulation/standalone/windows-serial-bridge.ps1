param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^COM\d+$')]
    [string]$Device,
    [Parameter(Mandatory = $true)]
    [ValidateRange(9600, 921600)]
    [int]$Baud
)

$ErrorActionPreference = 'Stop'

$source = @'
using System;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Threading;

public static class DominoSerialBridge
{
    public static int Run(string device, int baud)
    {
        using (var port = new SerialPort(device, baud, Parity.None, 8, StopBits.One))
        {
            port.Encoding = new UTF8Encoding(false);
            port.NewLine = "\n";
            port.ReadTimeout = 250;
            port.WriteTimeout = 500;
            port.DtrEnable = false;
            port.RtsEnable = false;
            port.Open();

            var running = true;
            var reader = new Thread(() =>
            {
                while (running)
                {
                    try
                    {
                        var line = port.ReadLine().TrimEnd('\r');
                        Console.Out.WriteLine(line);
                        Console.Out.Flush();
                    }
                    catch (TimeoutException) { }
                    catch (InvalidOperationException) { break; }
                    catch (IOException) { break; }
                }
            });
            reader.IsBackground = true;
            reader.Start();

            string command;
            while ((command = Console.In.ReadLine()) != null)
            {
                if (command.Length == 0) continue;
                port.WriteLine(command);
            }

            running = false;
            reader.Join(750);
        }
        return 0;
    }
}
'@

Add-Type -TypeDefinition $source -Language CSharp
exit [DominoSerialBridge]::Run($Device.ToUpperInvariant(), $Baud)
