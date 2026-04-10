package main

import (
	"context"
	"fmt"
	"os"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/utils/rpc"
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run() error {
	ctx := context.Background()
	logger := logging.NewLogger("cli")

	robot, err := client.New(ctx, os.Getenv("VIAM_ADDR"), logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			os.Getenv("VIAM_API_KEY_ID"),
			rpc.Credentials{Type: rpc.CredentialsTypeAPIKey, Payload: os.Getenv("VIAM_API_KEY")},
		)),
	)
	if err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer robot.Close(ctx)

	svc, err := generic.FromRobot(robot, "apriltag-arm")
	if err != nil {
		return fmt.Errorf("get service: %w", err)
	}

	// Example: list saved poses via DoCommand
	result, err := svc.DoCommand(ctx, map[string]interface{}{
		"command": "list_poses",
	})
	if err != nil {
		return fmt.Errorf("DoCommand: %w", err)
	}

	fmt.Println(result)
	return nil
}
