flowchart TD
    A([Start: Device Won't Turn On]) --> B{Is it plugged in?}
    
    B -- No --> C[Plug it in]
    B -- Yes --> D{Is the outlet switch ON?}
    
    C --> E[Try turning it on again]
    
    D -- No --> F[Turn on switch] --> E
    D -- Yes --> G[Call Tech Support]
    
    E --> H{Does it work now?}
    H -- Yes --> I([Success!])
    H -- No --> G
