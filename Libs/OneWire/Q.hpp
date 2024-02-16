#pragma once

template <typename T>
class OWQueue {
    class Node {
    public:
        T item;
        Node *next;

        Node() { next = nullptr; }

        ~Node() { next = nullptr; }
    };

    Node *head;
    Node *tail;
    unsigned int maxItems;
    unsigned int maxMemory;
    unsigned int count;

public:
    OWQueue(unsigned int maxItems = (unsigned int) -1, unsigned int maxMemory = (unsigned int) -1) {
        this->head = nullptr;
        this->tail = nullptr;
        this->count = 0;
        this->maxMemory = maxMemory;
        this->maxItems = maxMemory / sizeof(Node);

        if (maxItems != 0 && this->maxItems > maxItems) {
            this->maxItems = maxItems;
        }
    }

    ~OWQueue() {
        for (Node *node = head; node != nullptr; node = head) {
            head = node->next;
            delete node;
        }
    }

    /*
      Push an item to the queue.
      Returns false if memory is
      full, or true if the item
      was added to queue.
    */
    bool enqueue(T item) {
        if (count == maxItems) {
            return false;
        }

        Node *node = new Node;
        if (node == nullptr) {
            return false;
        }

        node->item = item;

        if (head == nullptr) {
            head = node;
            tail = node;
            count++;

            return true;
        }

        tail->next = node;
        tail = node;
        count++;

        return true;
    }

    /*
      Pop the front of the queue.
      Because exceptions are not
      usually implemented for
      microcontrollers, if queue
      is empty, a dummy item is
      returned.
    */
    T dequeue() {
        if ((count == 0) || (head == nullptr)) {
            return T();
        }

        Node *node = head;
        head = node->next;
        T item = node->item;
        delete node;
        node = nullptr;

        if (head == nullptr) {
            tail = nullptr;
        }

        count--;
        return item;
    }

    /*
      Returns true if the queue
      is empty, false otherwise.
    */
    bool isEmpty() { return head == nullptr; }

    /*
      Returns true if the queue
      is full, false otherwise.
    */
    bool isFull() { return count == maxItems; }

    /*
      Returns the number of items
      currently in the queue.
    */
    unsigned int itemCount() { return count; }

    /*
      Returns the size of the
      queue item in bytes.
    */
    unsigned int itemSize() { return sizeof(Node); }

    /*
      Returns the size of the queue
      (maximum number of items)
    */
    unsigned int maxQueueSize() { return maxItems; }

    /*
      Returns the size of the queue
      (maximum size in bytes)
    */
    unsigned int maxMemorySize() { return maxMemory; }

    /*
      Get the item in the front
      of the queue.
      Because exceptions are not
      usually implemented for
      microcontrollers, if queue
      is empty, a dummy item is
      returned.
    */
    T getHead() {
        if ((count == 0) || (head == nullptr)) {
            return T();
        }

        T item = head->item;
        return item;
    }

    /*
      Get the item in the back
      of the queue.
      Because exceptions are not
      usually implemented for
      microcontrollers, if queue
      is empty, a dummy item is
      returned.
    */
    T getTail() {
        if ((count == 0) || (head == nullptr)) {
            return T();
        }

        T item = tail->item;
        return item;
    }

    T *getHeadPtr() {
        if ((count == 0) || (head == nullptr)) {
            return nullptr;
        }

        return &(head->item);
    }

    T *getTailPtr() {
        if ((count == 0) || (head == nullptr)) {
            return nullptr;
        }

        return &(tail->item);
    }
};