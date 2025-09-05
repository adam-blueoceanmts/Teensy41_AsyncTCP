/****************************************************************************************************************************
  SyncClient_Impl.h

  Asynchronous TCP library for Espressif MCUs

  Copyright (c) 2016 Hristo Gochkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.

  Teensy41_AsyncTCP is a library for Teensy4.1 using LwIP-based QNEthernet

  Based on and modified from :

  1) ESPAsyncTCP    (https://github.com/me-no-dev/ESPAsyncTCP)
  2) AsyncTCP       (https://github.com/me-no-dev/AsyncTCP)

  Built by Khoi Hoang https://github.com/khoih-prog/Teensy41_AsyncTCP

  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
  as published bythe Free Software Foundation, either version 3 of the License, or (at your option) any later version.
  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

  Version: 1.1.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0    K Hoang     17/03/2022 Initial coding to support only Teensy4.1 using QNEthernet
  1.1.0    K Hoang     26/09/2022 Fix issue with slow browsers or network. Clean up. Remove hard-code if possible
 *****************************************************************************************************************************/

#pragma once

#ifndef _TEENSY41_ASYNC_TCP_SYNC_CLIENT_IMPL_H_
#define _TEENSY41_ASYNC_TCP_SYNC_CLIENT_IMPL_H_

#include "Arduino.h"
#include "SyncClient.hpp"

#include "Teensy41_AsyncTCP.hpp"

#include <algorithm>

/////////////////////////////////////////////////

#define DEBUG_ESP_SYNC_CLIENT

/////////////////////////////////////////////////

#if defined(DEBUG_ESP_SYNC_CLIENT) && !defined(SYNC_CLIENT_DEBUG)
  #define SYNC_CLIENT_DEBUG( format, ...) DEBUG_GENERIC_P("[SYNC_CLIENT]", format, ##__VA_ARGS__)
#endif

/////////////////////////////////////////////////

#ifndef SYNC_CLIENT_DEBUG
  #define SYNC_CLIENT_DEBUG(...) do { (void)0;} while(false)
#endif

/////////////////////////////////////////////////

/*
  Without LWIP_NETIF_TX_SINGLE_PBUF, all tcp_writes default to "no copy".
  Referenced data must be preserved and free-ed from the specified tcp_sent()
  callback. Alternative, tcp_writes need to use the TCP_WRITE_FLAG_COPY
  attribute.
*/
static_assert(LWIP_NETIF_TX_SINGLE_PBUF, "Required, tcp_write() must always copy.");

/////////////////////////////////////////////////

SyncClient::SyncClient(size_t txBufLen)
  : _client(NULL)
  , _tx_buffer()
  , _tx_buffer_head(0)
  , _tx_buffer_size(txBufLen)
  , _rx_buffer()
  , _rx_buffer_head(0)
  , _ref(NULL)
{
  _tx_buffer.reserve(txBufLen);
  ref();
}

/////////////////////////////////////////////////

SyncClient::SyncClient(AsyncClient *client, size_t txBufLen)
  : _client(client)
  , _tx_buffer()
  , _tx_buffer_head(0)
  , _tx_buffer_size(txBufLen)
  , _rx_buffer()
  , _rx_buffer_head(0)
  , _ref(NULL)
{
  _tx_buffer.reserve(txBufLen);
  if (ref() > 0 && _client != NULL)
    _attachCallbacks();
}

/////////////////////////////////////////////////

SyncClient::~SyncClient()
{
  if (0 == unref())
    _release();
}

/////////////////////////////////////////////////

void SyncClient::_release()
{
  if (_client != NULL)
  {
    _client->onData(NULL, NULL);
    _client->onAck(NULL, NULL);
    _client->onPoll(NULL, NULL);
    _client->abort();
    _client = NULL;
  }

  _tx_buffer.clear();
  _tx_buffer_head = 0;
  _rx_buffer.clear();
  _rx_buffer_head = 0;
}

/////////////////////////////////////////////////

int SyncClient::ref()
{
  if (_ref == NULL)
  {
    _ref = new (std::nothrow) int;

    if (_ref != NULL)
      *_ref = 0;
    else
      return -1;
  }

  return (++*_ref);
}

/////////////////////////////////////////////////

int SyncClient::unref()
{
  int count = -1;

  if (_ref != NULL)
  {
    count = --*_ref;

    if (0 == count)
    {
      delete _ref;
      _ref = NULL;
    }
  }

  return count;
}

/////////////////////////////////////////////////

#if ASYNC_TCP_SSL_ENABLED
  int SyncClient::_connect(const IPAddress& ip, uint16_t port, bool secure)
#else
  int SyncClient::_connect(const IPAddress& ip, uint16_t port)
#endif
{
  if (connected())
    return 0;

  if (_client != NULL)
    delete _client;

  _client = new (std::nothrow) AsyncClient();

  if (_client == NULL)
    return 0;

  _client->onConnect([](void *obj, AsyncClient * c)
  {
    ((SyncClient*)(obj))->_onConnect(c);
  }, this);

  _attachCallbacks_Disconnect();

#if ASYNC_TCP_SSL_ENABLED

  if (_client->connect(ip, port, secure))
#else
  if (_client->connect(ip, port))
#endif
  {
    while (_client != NULL && !_client->connected() && !_client->disconnecting())
      delay(1);

    return connected();
  }

  return 0;
}

/////////////////////////////////////////////////

#if ASYNC_TCP_SSL_ENABLED
  int SyncClient::connect(const char *host, uint16_t port, bool secure)
#else
  int SyncClient::connect(const char *host, uint16_t port)
#endif
{
  if (connected())
    return 0;

  if (_client != NULL)
    delete _client;

  _client = new (std::nothrow) AsyncClient();

  if (_client == NULL)
    return 0;

  _client->onConnect([](void *obj, AsyncClient * c)
  {
    ((SyncClient*)(obj))->_onConnect(c);
  }, this);

  _attachCallbacks_Disconnect();

#if ASYNC_TCP_SSL_ENABLED

  if (_client->connect(host, port, secure))
#else
  if (_client->connect(host, port))
#endif
  {
    while (_client != NULL && !_client->connected() && !_client->disconnecting())
      delay(1);

    return connected();
  }

  return 0;
}

/////////////////////////////////////////////////

//#define SYNCCLIENT_NEW_OPERATOR_EQUAL

#ifdef SYNCCLIENT_NEW_OPERATOR_EQUAL

/////////////////////////////////////////////////

/*
  New behavior for operator=

  Allow for the object to be placed on a queue and transfered to a new container
  with buffers still in tact. Avoiding receive data drops. Transfers rx and tx
  buffers. Supports return by value.

  Note, this is optional, the old behavior is the default.

*/
SyncClient & SyncClient::operator=(const SyncClient &other)
{
  int *rhsref = other._ref;
  ++*rhsref; // Just in case the left and right side are the same object with different containers

  if (0 == unref())
    _release();

  _ref = other._ref;
  ref();

  --*rhsref;
  // Transfer buffer contents and state
  _tx_buffer_size    = other._tx_buffer_size;
  _tx_buffer         = other._tx_buffer;
  _tx_buffer_head    = other._tx_buffer_head;
  _client            = other._client;
  _rx_buffer         = other._rx_buffer;
  _rx_buffer_head    = other._rx_buffer_head;

  if (_client)
    _attachCallbacks();

  return *this;
}

/////////////////////////////////////////////////

#else   // ! SYNCCLIENT_NEW_OPERATOR_EQUAL

/////////////////////////////////////////////////

// This is the origianl logic with null checks
SyncClient & SyncClient::operator=(const SyncClient &other)
{
  if (_client != NULL)
  {
    _client->abort();
    _client->free();
    _client = NULL;
  }

  _tx_buffer_size = other._tx_buffer_size;

  _tx_buffer.clear();
  _tx_buffer_head = 0;
  _rx_buffer.clear();
  _rx_buffer_head = 0;

  if (other._client != NULL)
    _client = other._client;

  _tx_buffer      = other._tx_buffer;
  _tx_buffer_head = other._tx_buffer_head;
  _rx_buffer      = other._rx_buffer;
  _rx_buffer_head = other._rx_buffer_head;

  if (_client)
    _attachCallbacks();

  return *this;
}

/////////////////////////////////////////////////

#endif

/////////////////////////////////////////////////

void SyncClient::setTimeout(uint32_t seconds)
{
  if (_client != NULL)
    _client->setRxTimeout(seconds);
}

/////////////////////////////////////////////////

uint8_t SyncClient::status()
{
  if (_client == NULL)
    return 0;

  return _client->state();
}

/////////////////////////////////////////////////

uint8_t SyncClient::connected()
{
  return (_client != NULL && _client->connected());
}

/////////////////////////////////////////////////

bool SyncClient::stop(unsigned int maxWaitMs)
{
  (void) maxWaitMs;

  if (_client != NULL)
    _client->close(true);

  return true;
}

/////////////////////////////////////////////////

size_t SyncClient::_sendBuffer()
{
  if (_client == NULL)
    return 0;

  if (!connected() || !_client->canSend() || (_tx_buffer_head >= _tx_buffer.size()))
    return 0;

  size_t sent_total = 0;

  while (connected() && _client->canSend() && (_tx_buffer_head < _tx_buffer.size()))
  {
    size_t available = _tx_buffer.size() - _tx_buffer_head;
    size_t sendable = _client->space();
    if (sendable < available)
      available = sendable;

    size_t sent = _client->write((const char*)(&_tx_buffer[_tx_buffer_head]), available, ASYNC_WRITE_FLAG_COPY);
    _tx_buffer_head += sent;
    sent_total += sent;

    if (sent != available)
      break;

    if (_tx_buffer_head == _tx_buffer.size())
    {
      _tx_buffer.clear();
      _tx_buffer_head = 0;
    }
  }

  return sent_total;
}

/////////////////////////////////////////////////

void SyncClient::_onData(void *data, size_t len)
{
  _client->ackLater();
  uint8_t *bytes = static_cast<uint8_t*>(data);
  try
  {
    _rx_buffer.insert(_rx_buffer.end(), bytes, bytes + len);
  }
  catch (...)
  {
    // Ran out of memory; abort to avoid data corruption
    _client->abort();
  }
}

/////////////////////////////////////////////////

void SyncClient::_onDisconnect()
{
  if (_client != NULL)
  {
    _client = NULL;
  }
  _tx_buffer.clear();
  _tx_buffer_head = 0;
}

/////////////////////////////////////////////////

void SyncClient::_onConnect(AsyncClient *c)
{
  _client = c;

  _tx_buffer.clear();
  _tx_buffer_head = 0;
  _tx_buffer.reserve(_tx_buffer_size);
  _attachCallbacks_AfterConnected();
}

/////////////////////////////////////////////////

void SyncClient::_attachCallbacks()
{
  _attachCallbacks_Disconnect();
  _attachCallbacks_AfterConnected();
}

/////////////////////////////////////////////////

void SyncClient::_attachCallbacks_AfterConnected()
{
  _client->onAck([](void *obj, AsyncClient * c, size_t len, uint32_t time)
  {
    (void) c;
    (void) len;
    (void) time;

    ((SyncClient*)(obj))->_sendBuffer();
  }, this);

  _client->onData([](void *obj, AsyncClient * c, void *data, size_t len)
  {
    (void) c;
    ((SyncClient*)(obj))->_onData(data, len);
  }, this);

  _client->onTimeout([](void *obj, AsyncClient * c, uint32_t time)
  {
    (void) obj;
    (void) time;
    c->close();
  }, this);
}

/////////////////////////////////////////////////

void SyncClient::_attachCallbacks_Disconnect()
{
  _client->onDisconnect([](void *obj, AsyncClient * c)
  {
    ((SyncClient*)(obj))->_onDisconnect();
    delete c;
  }, this);
}

/////////////////////////////////////////////////

size_t SyncClient::write(uint8_t data)
{
  return write(&data, 1);
}

/////////////////////////////////////////////////

size_t SyncClient::write(const uint8_t *data, size_t len)
{
  if (_client == NULL || !connected())
    return 0;

  size_t written = 0;

  while (written < len)
  {
    size_t buffered = _tx_buffer.size() - _tx_buffer_head;
    size_t space = (_tx_buffer_size > buffered) ? (_tx_buffer_size - buffered) : 0;

    if (space == 0)
    {
      while (connected() && !_client->canSend())
        delay(0);

      if (!connected())
        return written;

      _sendBuffer();
      buffered = _tx_buffer.size() - _tx_buffer_head;
      space = (_tx_buffer_size > buffered) ? (_tx_buffer_size - buffered) : 0;

      if (space == 0)
        break;
    }

    size_t toCopy = std::min(space, len - written);
    _tx_buffer.insert(_tx_buffer.end(), data + written, data + written + toCopy);
    written += toCopy;
  }

  if (connected() && _client->canSend())
    _sendBuffer();

  return written;
}

/////////////////////////////////////////////////

int SyncClient::available()
{
  return static_cast<int>(_rx_buffer.size() - _rx_buffer_head);
}

/////////////////////////////////////////////////

int SyncClient::peek()
{
  if (_rx_buffer_head >= _rx_buffer.size())
    return -1;
  return _rx_buffer[_rx_buffer_head];
}

/////////////////////////////////////////////////

int SyncClient::read(uint8_t *data, size_t len)
{
  size_t available = _rx_buffer.size() - _rx_buffer_head;
  if (available == 0)
    return -1;

  size_t toRead = std::min(len, available);
  std::copy(_rx_buffer.begin() + _rx_buffer_head,
            _rx_buffer.begin() + _rx_buffer_head + toRead,
            data);
  _rx_buffer_head += toRead;
  if (_rx_buffer_head == _rx_buffer.size())
  {
    _rx_buffer.clear();
    _rx_buffer_head = 0;
  }
  if (toRead && connected())
    _client->ack(toRead);
  return static_cast<int>(toRead);
}

/////////////////////////////////////////////////

int SyncClient::read()
{
  uint8_t res = 0;

  if (read(&res, 1) != 1)
    return -1;

  return res;
}

/////////////////////////////////////////////////

bool SyncClient::flush(unsigned int maxWaitMs)
{
  (void) maxWaitMs;

  if (_client == NULL || !connected())
    return false;

  if (_tx_buffer_head < _tx_buffer.size())
  {
    while (connected() && !_client->canSend())
      delay(0);

    if (_client == NULL)
      return false;

    _sendBuffer();
  }

  return (_tx_buffer_head == _tx_buffer.size());
}

/////////////////////////////////////////////////

#endif    // _TEENSY41_ASYNC_TCP_SYNC_CLIENT_IMPL_H_
