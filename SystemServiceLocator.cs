using System;
using System.Collections.Concurrent;
using Unity.Entities;
using UnityEngine;
using Utils;

namespace DefaultNamespace
{
    public abstract partial class ServiceSystemBase<T> : SystemBase,IServiceSystemLocator where T : class
    {
        protected override void OnCreate()
        {
           this.RegisterService<T>(this as T);
        }

        protected override void OnDestroy()
        {
            this.UnRegisterService<T>();
        }
    }

    public abstract class ServiceBehaviour<T> : MonoBehaviour, IServiceSystemLocator where T : class
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
            _instance = null;
            this.UnRegisterService<T>();
        }
    }

    
    public static class CanUnRegisterServiceSystem
    {
        public static void UnRegisterService<T>(this ICanUnRegisterServiceSystem _) where T : class
        {
            ISystemService.Unregister<T>();
        }
    }
    public static class CanRegisterServiceSystem
    {
        public static void RegisterService<T>(this ICanRegisterServiceSystem _,T service) where T : class
        {
            ISystemService.Register<T>(service);
        }
    }

    public static class CanGetServiceSystem
    {
        public static T GetService<T>(this ICanGetServiceSystem _) where T : class
        {
            return ISystemService.GetService<T>();
        }
    }

    public interface ICanRegisterServiceSystem{}
    public interface ICanUnRegisterServiceSystem{}
    public interface ICanGetServiceSystem{}
    public interface IServiceSystemLocator:ICanRegisterServiceSystem,ICanGetServiceSystem,ICanUnRegisterServiceSystem{}
    public interface ISystemService
    {
        private static readonly ConcurrentDictionary<Type, object> Services = new();

        public static void Register<T>(T service)
        {
            if (!Services.TryAdd(typeof(T), service))
                throw new Exception($"{service} registered failed");
            DebugSystem.Log($"<color=green>{typeof(T)}</color> registered successfully","乐酌");
        }

        public static void Unregister<T>()
        {
            if (!Services.TryRemove(typeof(T), out _))
                throw new InvalidOperationException($"{typeof(T)} unregister failed");
            DebugSystem.Log($"<color=red>{typeof(T)} </color>unRegistered successfully","乐酌");
        }

        public static T GetService<T>()
        {
            if (Services.TryGetValue(typeof(T), out object service))
                return (T)service;
            throw new NullReferenceException($"{typeof(T)} is not registered");
        }
    }
}