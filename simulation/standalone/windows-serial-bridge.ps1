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
            port.Handshake = Handshake.None;
            port.Open();

            int running = 1;
            int failed = 0;
            // Keep the blocking stdin read in a background thread. A USB read
            // failure must close the port even if Studio sends no more input.
            var writer = new Thread(() =>
            {
                try
                {
                    string command;
                    while (Interlocked.CompareExchange(ref running, 1, 1) == 1 &&
                           (command = Console.In.ReadLine()) != null)
                    {
                        if (command.Length != 0) port.WriteLine(command);
                    }
                }
                catch (Exception error)
                {
                    Console.Error.WriteLine(error.Message);
                    Interlocked.Exchange(ref failed, 1);
                }
                finally { Interlocked.Exchange(ref running, 0); }
            });
            writer.IsBackground = true;
            writer.Start();

            try
            {
                while (Interlocked.CompareExchange(ref running, 1, 1) == 1)
                {
                    try
                    {
                        Console.Out.WriteLine(port.ReadLine().TrimEnd('\r'));
                        Console.Out.Flush();
                    }
                    catch (TimeoutException) { }
                }
            }
            catch (Exception error)
            {
                Console.Error.WriteLine(error.Message);
                Interlocked.Exchange(ref failed, 1);
            }
            finally
            {
                Interlocked.Exchange(ref running, 0);
                port.Close();
                writer.Join(750);
            }
            return failed;
        }
    }
}
'@

Add-Type -TypeDefinition $source -Language CSharp
exit [DominoSerialBridge]::Run($Device.ToUpperInvariant(), $Baud)
