```mermaid
graph TD
    A([Start]) --> B{Does it have power?}
    
    B -- No --> C[Plug it in]
    B -- Yes --> D{Is the outlet switch ON?}

    C --> E[Try turning it on again]

    D -- No --> F[Turn on switch] --> E
    D -- Yes --> G[Call Tech Support]

    E --> H{Does it work now?}
    H -- Yes --> I([Success!])
    H -- No --> G


