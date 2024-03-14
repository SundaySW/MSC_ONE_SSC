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

    bool isEmpty() { return head == nullptr; }

    bool isFull() { return count == maxItems; }

    unsigned int itemCount() { return count; }

    unsigned int itemSize() { return sizeof(Node); }

    unsigned int maxQueueSize() { return maxItems; }


    unsigned int maxMemorySize() { return maxMemory; }

    T getHead() {
        if ((count == 0) || (head == nullptr)) {
            return T();
        }

        T item = head->item;
        return item;
    }

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