#pragma once
#include <unordered_map>
#include <list>
#include <memory>

template<class Key, class T>
class LRUCache {
    int capacity;
    std::list<Key> order;
    std::shared_ptr<std::mutex> order_mtx; // ensure the get function can be call thread safely
    std::unordered_map<Key, std::pair<std::shared_ptr<T>, typename std::list<Key>::iterator>> cache;

public:
    LRUCache(int capacity) : capacity(capacity) {order_mtx = std::make_shared<std::mutex>();}
    void erase(const Key& key) {
      auto it = cache.find(key);
      if (it != cache.end()) {
        std::lock_guard<std::mutex> lock(*order_mtx);
        order.erase(it->second.second);
        cache.erase(it);
      }
    }
    std::shared_ptr<T> get(const Key& key) {
      auto it = cache.find(key);
      if (it == cache.end()) return nullptr;
      {
        std::lock_guard<std::mutex> lock(*order_mtx);
        order.erase(it->second.second);
        order.push_back(key);
        it->second.second = --order.end();
      }
      return it->second.first;
    }
    void put(const Key& key, const std::shared_ptr<T>& value) {
      auto it = cache.find(key);
      if (it != cache.end()) {
        {
          std::lock_guard<std::mutex> lock(*order_mtx);
          order.erase(it->second.second);
          order.push_back(key);
          it->second.second = --order.end();
        }
        return;
      }
      std::lock_guard<std::mutex> lock(*order_mtx);
      if (cache.size() >= capacity) {
        Key old = order.front();
        order.pop_front();
        cache.erase(old);
      }
      order.push_back(key);
      cache[key] = {value, --order.end()};
    }
    bool empty() {
      std::lock_guard<std::mutex> lock(*order_mtx);
      return cache.size() == 0;
    }
};