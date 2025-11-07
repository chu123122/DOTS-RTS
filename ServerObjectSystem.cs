using System;
using System.Collections.Concurrent;
using UnityEngine;
using Utils;

namespace 简单战斗.ServiceLocator
{
    public static class CanGetService
    {
        public static T GetServiceObject<T>(this IGetService getService) where T : class
        {
            return (T)IService.Get(typeof(T));
        }
    }

    public static class CanRegisterService
    {
        public static void RegisterService(this IRegisterService registerService, MonoBehaviour service)
        {
            // 获取该类实现的所有接口
            var interfaces = service.GetType().GetInterfaces();

            // 注册继承自IServiceBase的接口
            foreach (var serviceInterface in interfaces)
            {
                //如果接口/非接口的实例已经存储
                if (IService.HaveRegistered(serviceInterface) || IService.HaveRegistered(service.GetType())) return;

                if (typeof(IServiceBase).IsAssignableFrom(serviceInterface) && serviceInterface != typeof(IServiceBase))
                {
                    IService.Add(serviceInterface, service);
                    DebugSystem.Log($"Service {service.GetType().Name} registered as {serviceInterface.Name}","乐酌");
                    return;
                }
            }

            if (IService.HaveRegistered(service.GetType())) return;
            IService.Add(service.GetType(), service);
            DebugSystem.Log($"Service {service.GetType().Name} registered as {service.GetType().Name}","乐酌");
        }
    }

    public interface IGetService
    {
    }

    public interface IRegisterService
    {
    }

    public interface IServiceBase
    {
    }

    public class ServiceObject<T> : MonoBehaviour, IRegisterService where T : class
    {
        private static T _instance;
        public static T Instance => _instance;

        protected virtual void Awake()
        {
            if (_instance == null)
                _instance = this as T;
            else
                Destroy(this);

            this.RegisterService(this);
        }
        
        protected virtual void OnDestroy()
        {
            if (_instance == this as T)
            {
                _instance = null;
                IService.Remove(this.GetType(),this);
            }
        }
    }

    public interface IService
    {
        public static readonly ConcurrentDictionary<Type, object> ServiceDic = new ConcurrentDictionary<Type, object>();

        public static void Add(Type service, object instance)
        {
            if (!ServiceDic.TryAdd(service, instance))
                throw new Exception($"{service} already registered");
        }

        public static object Get(Type service)
        {
            if (ServiceDic.TryGetValue(service, out object instance))
                return instance;
            throw new NullReferenceException($"{service} is not registered");
        }

        public static void Remove(Type service,MonoBehaviour serviceMono)
        {
            if(ServiceDic.TryRemove(service,out object instance))return;
            var interfaces = serviceMono.GetType().GetInterfaces();
            foreach (var @serviceInterface in interfaces)
            {
                if (typeof(IServiceBase).IsAssignableFrom(serviceInterface) && serviceInterface != typeof(IServiceBase))
                {
                    if(ServiceDic.TryRemove(serviceInterface,out object instance1))return;
                }
            }
            throw new Exception($"{service} removed failed");
        }

        public static bool HaveRegistered(Type service)
        {
            return ServiceDic.ContainsKey(service);
        }
    }

    public interface IServiceSystem : IGetService, IRegisterService, IService
    {
    }
}