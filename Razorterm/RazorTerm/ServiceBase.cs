using System;
using System.Threading;
using System.Threading.Tasks;
using RazorTerm.Logging;

namespace RazorTerm
{
    public abstract class ServiceBase
    {
        protected bool Running;
        private Thread _runnerThread;

        public void Start()
        {
            if (Running || _runnerThread?.ThreadState == ThreadState.Running)
            {
                throw new ApplicationException($"{GetType().Name} service already running");
            }

            Running = true;
            _runnerThread = new Thread(() => ExceptionHandledStarter().Wait())
            {
                IsBackground = true
            };
            _runnerThread.Start();
        }

        public async Task Stop()
        {
            Logger.Log($"Stopping {GetType().Name} service...");

            Running = false;

            try
            {
                var stopTask = OnStop();
                if (await Task.WhenAny(stopTask, Task.Delay(TimeSpan.FromSeconds(10))) != stopTask)
                {
                    ForceStop();
                }

                Logger.Log($"{GetType().Name} service stopped");
            }
            catch (Exception e)
            {
                Logger.Log($"Exception when stopping {GetType().Name} service");
                Logger.Log(e);
            }
            finally
            {
                ForceStop();
                _runnerThread = null;
            }
        }

        private void ForceStop()
        {
            if (_runnerThread?.ThreadState == ThreadState.Running)
            {
                Logger.Log($"Forcing {GetType().Name} service to stop...");
                _runnerThread.Abort();
            }
        }

        private async Task ExceptionHandledStarter()
        {
            var success = false;
            try
            {
                await OnStart();
                success = true;
            }
            catch (Exception e)
            {
                success = false;
                Logger.Log($"Uncaught exception in {GetType().Name} service starter");
                Logger.Log(e);
            }

            if (success)
            {
                await ExceptionHandledRunner();
            }
        }

        private async Task ExceptionHandledRunner()
        {
            try
            {
                await Run();
            }
            catch (Exception e)
            {
                Logger.Log($"Uncaught exception in {GetType().Name} service runner");
                Logger.Log(e);

                await Task.Delay(1000);

                if (Running)
                {
                    await ExceptionHandledRunner();
                }
            }
        }

        protected abstract Task OnStart();
        protected abstract Task Run();
        protected abstract Task OnStop();
    }
}