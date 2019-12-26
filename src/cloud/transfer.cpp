#include "cloud/transfer.h"

#include "net/http_response_parser.h"
#include "util/optional.h"

using namespace std;
using namespace chrono;
using namespace pbrt;

const static std::string UNSIGNED_PAYLOAD = "UNSIGNED-PAYLOAD";

TransferAgent::TransferAgent(const S3StorageBackend& backend) {
    clientConfig.credentials = backend.client().credentials();
    clientConfig.region = backend.client().config().region;
    clientConfig.bucket = backend.bucket();
    clientConfig.prefix = backend.prefix();

    clientConfig.endpoint =
        S3::endpoint(clientConfig.region, clientConfig.bucket);

    clientConfig.address = Address{clientConfig.endpoint, "http"};

    for (size_t i = 0; i < MAX_THREADS; i++) {
        threads.emplace_back(&TransferAgent::workerThread, this, i);
    }
}

TransferAgent::~TransferAgent() {
    terminated = true;
    cv.notify_all();
    for (auto& t : threads) t.join();
}

HTTPRequest TransferAgent::getRequest(const Action& action) {
    switch (action.type) {
    case Action::Upload:
        return S3PutRequest(clientConfig.credentials, clientConfig.endpoint,
                            clientConfig.region, action.key, action.data,
                            UNSIGNED_PAYLOAD)
            .to_http_request();

    case Action::Download:
        return S3GetRequest(clientConfig.credentials, clientConfig.endpoint,
                            clientConfig.region, action.key)
            .to_http_request();
    default:
        throw runtime_error("Unknown action type");
    }
}

#define TRY_OPERATION(x)                     \
    try {                                    \
        x;                                   \
    } catch (exception & ex) {               \
        this_thread::sleep_for(2 * backoff); \
        tryCount++;                          \
        connectionOkay = false;              \
        continue;                            \
    }

void TransferAgent::workerThread(const size_t threadId) {
    constexpr milliseconds backoff{50};
    size_t tryCount = 0;

    Optional<Action> action;

    while (!terminated) {
        TCPSocket s3;
        auto parser = make_unique<HTTPResponseParser>();
        bool connectionOkay = true;
        size_t requestCount = 0;

        if (tryCount > 0) {
            tryCount = min<size_t>(tryCount, 7u);
            this_thread::sleep_for(backoff * (1 << (tryCount - 1)));
        }

        TRY_OPERATION(s3.connect(clientConfig.address));

        while (!terminated && connectionOkay) {
            /* make sure we have an action to perfom */
            if (!action.initialized()) {
                unique_lock<mutex> lock{outstandingMutex};

                cv.wait(lock, [this]() {
                    return terminated || !outstanding.empty();
                });

                if (terminated) return;

                action.initialize(move(outstanding.front()));
                outstanding.pop();
            }

            HTTPRequest request = getRequest(*action);
            parser->new_request_arrived(request);

            TRY_OPERATION(s3.write(request.str()));
            requestCount++;

            while (!terminated && connectionOkay && action.initialized()) {
                TRY_OPERATION(parser->parse(s3.read()));

                if (!parser->empty()) {
                    const string status = move(parser->front().status_code());
                    const string data = move(parser->front().body());
                    parser->pop();

                    switch (status[0]) {
                    case '2':  // successful
                        action->data = move(data);

                        {
                            unique_lock<mutex> lock{resultsMutex};
                            results.emplace(move(*action));
                        }

                        tryCount = 0;
                        action.clear();
                        eventFD.write_event();
                        break;

                    case '5':  // we need to slow down
                        connectionOkay = false;
                        tryCount++;
                        break;

                    default:  // unexpected response, like 404 or something
                        throw runtime_error("transfer failed: " + status);
                    }
                }
            }

            if (requestCount >= MAX_REQUESTS_ON_CONNECTION) {
                connectionOkay = false;
            }
        }
    }
}

void TransferAgent::doAction(Action&& action) {
    {
        unique_lock<mutex> lock{outstandingMutex};
        outstanding.push(move(action));
    }

    cv.notify_one();
    return;
}

uint64_t TransferAgent::requestDownload(const string& key) {
    doAction({nextId, Action::Download, key, string()});
    return nextId++;
}

uint64_t TransferAgent::requestUpload(const string& key, string&& data) {
    doAction({nextId, Action::Upload, key, move(data)});
    return nextId++;
}

Optional<TransferAgent::Action> TransferAgent::try_pop() {
    unique_lock<mutex> lock{resultsMutex};

    if (results.empty()) return {};

    Action action = move(results.front());
    results.pop();

    return {true, move(action)};
}
