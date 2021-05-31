//
// Copyright (c) ZeroC, Inc. All rights reserved.
//
//
// Ice version 3.7.3
//
// <auto-generated>
//
// Generated from file `CoppeliaUtils.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __CoppeliaUtils_h__
#define __CoppeliaUtils_h__

#include <IceUtil/PushDisableWarnings.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/ValueF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Comparable.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/GCObject.h>
#include <Ice/Value.h>
#include <Ice/Incoming.h>
#include <Ice/FactoryTableInit.h>
#include <IceUtil/ScopedArray.h>
#include <Ice/Optional.h>
#include <IceUtil/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 >= 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 3
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace RoboCompCoppeliaUtils
{

class CoppeliaUtils;
class CoppeliaUtilsPrx;

}

namespace RoboCompCoppeliaUtils
{

struct PoseType
{
    float x;
    float y;
    float z;
    float rx;
    float ry;
    float rz;

    /**
     * Obtains a tuple containing all of the struct's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const float&, const float&, const float&, const float&, const float&, const float&> ice_tuple() const
    {
        return std::tie(x, y, z, rx, ry, rz);
    }
};

enum class TargetTypes : unsigned char
{
    Info,
    Hand,
    HeadCamera
};

using Ice::operator<;
using Ice::operator<=;
using Ice::operator>;
using Ice::operator>=;
using Ice::operator==;
using Ice::operator!=;

}

namespace RoboCompCoppeliaUtils
{

class CoppeliaUtils : public virtual ::Ice::Object
{
public:

    using ProxyType = CoppeliaUtilsPrx;

    /**
     * Determines whether this object supports an interface with the given Slice type ID.
     * @param id The fully-scoped Slice type ID.
     * @param current The Current object for the invocation.
     * @return True if this object supports the interface, false, otherwise.
     */
    virtual bool ice_isA(::std::string id, const ::Ice::Current& current) const override;

    /**
     * Obtains a list of the Slice type IDs representing the interfaces supported by this object.
     * @param current The Current object for the invocation.
     * @return A list of fully-scoped type IDs.
     */
    virtual ::std::vector<::std::string> ice_ids(const ::Ice::Current& current) const override;

    /**
     * Obtains a Slice type ID representing the most-derived interface supported by this object.
     * @param current The Current object for the invocation.
     * @return A fully-scoped type ID.
     */
    virtual ::std::string ice_id(const ::Ice::Current& current) const override;

    /**
     * Obtains the Slice type ID corresponding to this class.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    virtual void addOrModifyDummy(TargetTypes type, ::std::string name, PoseType pose, const ::Ice::Current& current) = 0;
    /// \cond INTERNAL
    bool _iceD_addOrModifyDummy(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&) override;
    /// \endcond
};

}

namespace RoboCompCoppeliaUtils
{

class CoppeliaUtilsPrx : public virtual ::Ice::Proxy<CoppeliaUtilsPrx, ::Ice::ObjectPrx>
{
public:

    void addOrModifyDummy(TargetTypes type, const ::std::string& name, const PoseType& pose, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        _makePromiseOutgoing<void>(true, this, &CoppeliaUtilsPrx::_iceI_addOrModifyDummy, type, name, pose, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto addOrModifyDummyAsync(TargetTypes type, const ::std::string& name, const PoseType& pose, const ::Ice::Context& context = ::Ice::noExplicitContext)
        -> decltype(::std::declval<P<void>>().get_future())
    {
        return _makePromiseOutgoing<void, P>(false, this, &CoppeliaUtilsPrx::_iceI_addOrModifyDummy, type, name, pose, context);
    }

    ::std::function<void()>
    addOrModifyDummyAsync(TargetTypes type, const ::std::string& name, const PoseType& pose,
                          ::std::function<void()> response,
                          ::std::function<void(::std::exception_ptr)> ex = nullptr,
                          ::std::function<void(bool)> sent = nullptr,
                          const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<void>(response, ex, sent, this, &RoboCompCoppeliaUtils::CoppeliaUtilsPrx::_iceI_addOrModifyDummy, type, name, pose, context);
    }

    /// \cond INTERNAL
    void _iceI_addOrModifyDummy(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<void>>&, TargetTypes, const ::std::string&, const PoseType&, const ::Ice::Context&);
    /// \endcond

    /**
     * Obtains the Slice type ID of this interface.
     * @return The fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:

    /// \cond INTERNAL
    CoppeliaUtilsPrx() = default;
    friend ::std::shared_ptr<CoppeliaUtilsPrx> IceInternal::createProxy<CoppeliaUtilsPrx>();

    virtual ::std::shared_ptr<::Ice::ObjectPrx> _newInstance() const override;
    /// \endcond
};

}

/// \cond STREAM
namespace Ice
{

template<>
struct StreamableTraits<::RoboCompCoppeliaUtils::PoseType>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 24;
    static const bool fixedLength = true;
};

template<typename S>
struct StreamReader<::RoboCompCoppeliaUtils::PoseType, S>
{
    static void read(S* istr, ::RoboCompCoppeliaUtils::PoseType& v)
    {
        istr->readAll(v.x, v.y, v.z, v.rx, v.ry, v.rz);
    }
};

template<>
struct StreamableTraits< ::RoboCompCoppeliaUtils::TargetTypes>
{
    static const StreamHelperCategory helper = StreamHelperCategoryEnum;
    static const int minValue = 0;
    static const int maxValue = 2;
    static const int minWireSize = 1;
    static const bool fixedLength = false;
};

}
/// \endcond

/// \cond INTERNAL
namespace RoboCompCoppeliaUtils
{

using CoppeliaUtilsPtr = ::std::shared_ptr<CoppeliaUtils>;
using CoppeliaUtilsPrxPtr = ::std::shared_ptr<CoppeliaUtilsPrx>;

}
/// \endcond

#else // C++98 mapping

namespace IceProxy
{

namespace RoboCompCoppeliaUtils
{

class CoppeliaUtils;
/// \cond INTERNAL
void _readProxy(::Ice::InputStream*, ::IceInternal::ProxyHandle< CoppeliaUtils>&);
::IceProxy::Ice::Object* upCast(CoppeliaUtils*);
/// \endcond

}

}

namespace RoboCompCoppeliaUtils
{

class CoppeliaUtils;
/// \cond INTERNAL
::Ice::Object* upCast(CoppeliaUtils*);
/// \endcond
typedef ::IceInternal::Handle< CoppeliaUtils> CoppeliaUtilsPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils> CoppeliaUtilsPrx;
typedef CoppeliaUtilsPrx CoppeliaUtilsPrxPtr;
/// \cond INTERNAL
void _icePatchObjectPtr(CoppeliaUtilsPtr&, const ::Ice::ObjectPtr&);
/// \endcond

}

namespace RoboCompCoppeliaUtils
{

struct PoseType
{
    ::Ice::Float x;
    ::Ice::Float y;
    ::Ice::Float z;
    ::Ice::Float rx;
    ::Ice::Float ry;
    ::Ice::Float rz;
};

enum TargetTypes
{
    Info,
    Hand,
    HeadCamera
};

}

namespace RoboCompCoppeliaUtils
{

/**
 * Base class for asynchronous callback wrapper classes used for calls to
 * IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 * Create a wrapper instance by calling ::RoboCompCoppeliaUtils::newCallback_CoppeliaUtils_addOrModifyDummy.
 */
class Callback_CoppeliaUtils_addOrModifyDummy_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_CoppeliaUtils_addOrModifyDummy_Base> Callback_CoppeliaUtils_addOrModifyDummyPtr;

}

namespace IceProxy
{

namespace RoboCompCoppeliaUtils
{

class CoppeliaUtils : public virtual ::Ice::Proxy<CoppeliaUtils, ::IceProxy::Ice::Object>
{
public:

    void addOrModifyDummy(::RoboCompCoppeliaUtils::TargetTypes type, const ::std::string& name, const ::RoboCompCoppeliaUtils::PoseType& pose, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        end_addOrModifyDummy(_iceI_begin_addOrModifyDummy(type, name, pose, context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_addOrModifyDummy(::RoboCompCoppeliaUtils::TargetTypes type, const ::std::string& name, const ::RoboCompCoppeliaUtils::PoseType& pose, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_addOrModifyDummy(type, name, pose, context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_addOrModifyDummy(::RoboCompCoppeliaUtils::TargetTypes type, const ::std::string& name, const ::RoboCompCoppeliaUtils::PoseType& pose, const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_addOrModifyDummy(type, name, pose, ::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_addOrModifyDummy(::RoboCompCoppeliaUtils::TargetTypes type, const ::std::string& name, const ::RoboCompCoppeliaUtils::PoseType& pose, const ::Ice::Context& context, const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_addOrModifyDummy(type, name, pose, context, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_addOrModifyDummy(::RoboCompCoppeliaUtils::TargetTypes type, const ::std::string& name, const ::RoboCompCoppeliaUtils::PoseType& pose, const ::RoboCompCoppeliaUtils::Callback_CoppeliaUtils_addOrModifyDummyPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_addOrModifyDummy(type, name, pose, ::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_addOrModifyDummy(::RoboCompCoppeliaUtils::TargetTypes type, const ::std::string& name, const ::RoboCompCoppeliaUtils::PoseType& pose, const ::Ice::Context& context, const ::RoboCompCoppeliaUtils::Callback_CoppeliaUtils_addOrModifyDummyPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_addOrModifyDummy(type, name, pose, context, cb, cookie);
    }

    void end_addOrModifyDummy(const ::Ice::AsyncResultPtr& result);

private:

    ::Ice::AsyncResultPtr _iceI_begin_addOrModifyDummy(::RoboCompCoppeliaUtils::TargetTypes, const ::std::string&, const ::RoboCompCoppeliaUtils::PoseType&, const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

public:

    /**
     * Obtains the Slice type ID corresponding to this interface.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:
    /// \cond INTERNAL

    virtual ::IceProxy::Ice::Object* _newInstance() const;
    /// \endcond
};

}

}

namespace RoboCompCoppeliaUtils
{

class CoppeliaUtils : public virtual ::Ice::Object
{
public:

    typedef CoppeliaUtilsPrx ProxyType;
    typedef CoppeliaUtilsPtr PointerType;

    virtual ~CoppeliaUtils();

    /**
     * Determines whether this object supports an interface with the given Slice type ID.
     * @param id The fully-scoped Slice type ID.
     * @param current The Current object for the invocation.
     * @return True if this object supports the interface, false, otherwise.
     */
    virtual bool ice_isA(const ::std::string& id, const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains a list of the Slice type IDs representing the interfaces supported by this object.
     * @param current The Current object for the invocation.
     * @return A list of fully-scoped type IDs.
     */
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains a Slice type ID representing the most-derived interface supported by this object.
     * @param current The Current object for the invocation.
     * @return A fully-scoped type ID.
     */
    virtual const ::std::string& ice_id(const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains the Slice type ID corresponding to this class.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    virtual void addOrModifyDummy(TargetTypes type, const ::std::string& name, const PoseType& pose, const ::Ice::Current& current = ::Ice::emptyCurrent) = 0;
    /// \cond INTERNAL
    bool _iceD_addOrModifyDummy(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

protected:

    /// \cond STREAM
    virtual void _iceWriteImpl(::Ice::OutputStream*) const;
    virtual void _iceReadImpl(::Ice::InputStream*);
    /// \endcond
};

/// \cond INTERNAL
inline bool operator==(const CoppeliaUtils& lhs, const CoppeliaUtils& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) == static_cast<const ::Ice::Object&>(rhs);
}

inline bool operator<(const CoppeliaUtils& lhs, const CoppeliaUtils& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) < static_cast<const ::Ice::Object&>(rhs);
}
/// \endcond

}

/// \cond STREAM
namespace Ice
{

template<>
struct StreamableTraits< ::RoboCompCoppeliaUtils::PoseType>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 24;
    static const bool fixedLength = true;
};

template<typename S>
struct StreamWriter< ::RoboCompCoppeliaUtils::PoseType, S>
{
    static void write(S* ostr, const ::RoboCompCoppeliaUtils::PoseType& v)
    {
        ostr->write(v.x);
        ostr->write(v.y);
        ostr->write(v.z);
        ostr->write(v.rx);
        ostr->write(v.ry);
        ostr->write(v.rz);
    }
};

template<typename S>
struct StreamReader< ::RoboCompCoppeliaUtils::PoseType, S>
{
    static void read(S* istr, ::RoboCompCoppeliaUtils::PoseType& v)
    {
        istr->read(v.x);
        istr->read(v.y);
        istr->read(v.z);
        istr->read(v.rx);
        istr->read(v.ry);
        istr->read(v.rz);
    }
};

template<>
struct StreamableTraits< ::RoboCompCoppeliaUtils::TargetTypes>
{
    static const StreamHelperCategory helper = StreamHelperCategoryEnum;
    static const int minValue = 0;
    static const int maxValue = 2;
    static const int minWireSize = 1;
    static const bool fixedLength = false;
};

}
/// \endcond

namespace RoboCompCoppeliaUtils
{

/**
 * Type-safe asynchronous callback wrapper class used for calls to
 * IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 * Create a wrapper instance by calling ::RoboCompCoppeliaUtils::newCallback_CoppeliaUtils_addOrModifyDummy.
 */
template<class T>
class CallbackNC_CoppeliaUtils_addOrModifyDummy : public Callback_CoppeliaUtils_addOrModifyDummy_Base, public ::IceInternal::OnewayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)();

    CallbackNC_CoppeliaUtils_addOrModifyDummy(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallbackNC<T>(obj, cb, excb, sentcb)
    {
    }
};

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 */
template<class T> Callback_CoppeliaUtils_addOrModifyDummyPtr
newCallback_CoppeliaUtils_addOrModifyDummy(const IceUtil::Handle<T>& instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_CoppeliaUtils_addOrModifyDummy<T>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 */
template<class T> Callback_CoppeliaUtils_addOrModifyDummyPtr
newCallback_CoppeliaUtils_addOrModifyDummy(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_CoppeliaUtils_addOrModifyDummy<T>(instance, 0, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 */
template<class T> Callback_CoppeliaUtils_addOrModifyDummyPtr
newCallback_CoppeliaUtils_addOrModifyDummy(T* instance, void (T::*cb)(), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_CoppeliaUtils_addOrModifyDummy<T>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 */
template<class T> Callback_CoppeliaUtils_addOrModifyDummyPtr
newCallback_CoppeliaUtils_addOrModifyDummy(T* instance, void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_CoppeliaUtils_addOrModifyDummy<T>(instance, 0, excb, sentcb);
}

/**
 * Type-safe asynchronous callback wrapper class with cookie support used for calls to
 * IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 * Create a wrapper instance by calling ::RoboCompCoppeliaUtils::newCallback_CoppeliaUtils_addOrModifyDummy.
 */
template<class T, typename CT>
class Callback_CoppeliaUtils_addOrModifyDummy : public Callback_CoppeliaUtils_addOrModifyDummy_Base, public ::IceInternal::OnewayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const CT&);

    Callback_CoppeliaUtils_addOrModifyDummy(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::OnewayCallback<T, CT>(obj, cb, excb, sentcb)
    {
    }
};

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 */
template<class T, typename CT> Callback_CoppeliaUtils_addOrModifyDummyPtr
newCallback_CoppeliaUtils_addOrModifyDummy(const IceUtil::Handle<T>& instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_CoppeliaUtils_addOrModifyDummy<T, CT>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 */
template<class T, typename CT> Callback_CoppeliaUtils_addOrModifyDummyPtr
newCallback_CoppeliaUtils_addOrModifyDummy(const IceUtil::Handle<T>& instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_CoppeliaUtils_addOrModifyDummy<T, CT>(instance, 0, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 */
template<class T, typename CT> Callback_CoppeliaUtils_addOrModifyDummyPtr
newCallback_CoppeliaUtils_addOrModifyDummy(T* instance, void (T::*cb)(const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_CoppeliaUtils_addOrModifyDummy<T, CT>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompCoppeliaUtils::CoppeliaUtils::begin_addOrModifyDummy.
 */
template<class T, typename CT> Callback_CoppeliaUtils_addOrModifyDummyPtr
newCallback_CoppeliaUtils_addOrModifyDummy(T* instance, void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_CoppeliaUtils_addOrModifyDummy<T, CT>(instance, 0, excb, sentcb);
}

}

#endif

#include <IceUtil/PopDisableWarnings.h>
#endif
